"""Tests of the laptop tool, partly against fhsim (the firmware core on a simulated head): pytest tool/tests"""
import datetime as dt
import os
import re
import struct
import subprocess
import sys
import time


ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'tool'))
from fhs import proto, store  # noqa: E402

FHSIM = os.path.join(ROOT, 'firmware', 'host', 'fhsim')
FHS = [sys.executable, '-m', 'fhs']


def fhs(env, *args, check=True, timeout=120):
    p = subprocess.run(FHS + list(args), env=env, capture_output=True, text=True, timeout=timeout)
    if check and p.returncode:
        raise AssertionError('fhs %s failed:\n%s\n%s' % (' '.join(args), p.stdout, p.stderr))
    return p


def start_sim(tmp_path, scenario, *extra, ble=True):
    sc = tmp_path / 'scenario.txt'
    sc.write_text(scenario)
    usb, bl = tmp_path / 'usb', tmp_path / 'ble'
    cmd = [FHSIM, '--scenario', str(sc), '--usb', str(usb), '--quiet'] + (['--ble', str(bl)] if ble else []) + list(extra)
    p = subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True)
    for _ in range(100):
        if usb.exists() and (not ble or bl.exists()):
            break
        time.sleep(0.05)
    return p, str(usb), str(bl)


def stop(p):
    p.terminate()
    p.wait(10)


def run_sim(tmp_path, scenario, *extra):
    sc = tmp_path / 'scenario.txt'
    sc.write_text(scenario)
    subprocess.run([FHSIM, '--scenario', str(sc), '--batch', '--speed', '0', '--quiet'] + list(extra), check=True, timeout=600)


# ---------- formats ----------

def test_crc_and_record():
    assert proto.crc16(b'123456789') == 0x29B1
    raw = struct.pack('<hhHHHHBBH', -4512, 1234, 55, 300, 900, 120, 0x80 | 0x37, proto.F_UPRIGHT | proto.F_CLOCK, 7)
    r = proto.Record.unpack(raw)
    assert (r.roll, r.pitch, r.move, r.buzz, r.mic, r.snore) == (-45.12, 12.34, 5.5, 30.0, 900, 120)
    assert r.shadow and r.would == 3 and r.variant == 7 and r.motor == 0   # step 3, variant 7, shadow
    assert r.flag_names() == ['upright', 'clock']
    therapy = proto.Record.unpack(raw[:12] + bytes([0x4c]) + raw[13:])
    assert not therapy.shadow and therapy.motor == 4 and therapy.would == 4 and therapy.variant == 12
    cue = proto.Record.unpack(raw[:12] + bytes([0x61]) + raw[13:])   # the bedtime buzz, battery low: ran, no nudge
    assert cue.motor == 6 and cue.would == 0 and cue.variant == 1
    cue = proto.Record.unpack(raw[:12] + bytes([0x61]) + raw[13:])   # the bedtime buzz, battery low: ran, not a nudge
    assert not cue.shadow and cue.motor == 6 and cue.would == 0 and cue.variant == 1
    old = proto.Record.unpack(raw[:12] + bytes([0x80 | 50]) + raw[13:], version=1)   # format 1: a motor level
    assert old.shadow and old.would == 2 and old.variant == 0
    assert old.roll == 45.12 and proto.Record.unpack(raw, version=2).roll == 45.12   # before format 3: sign flipped
    line = 'R 3 7 ' + raw.hex()
    assert proto.parse_R(line)[:2] == (3, 7)
    assert proto.parse_R('R 3 8 ' + raw.hex()) is None   # seq doesn't match the record
    assert proto.parse_R('R 3 7 abc') is None


def make_block(block_no, boot, first_seq, n, t0_ms=0, closed=True):
    h = struct.pack('<IIIIqH3hBBBBBB', proto.BLOCK_MAGIC, block_no, boot, first_seq, t0_ms, 3900, 0, 0, 10000,
                    proto.FORMAT_VERSION, 16, 253, 0, 0, 0xFF)
    h += struct.pack('<HH', proto.crc16(h), n if closed else 0xFFFF) + b'\xff' * 6
    recs = b''.join(struct.pack('<hhHHHHBBH', i, 0, 0, 0, 0, 0, 0x80, 0, (first_seq + i) & 0xFFFF) for i in range(n))
    return (h + recs).ljust(proto.BLOCK_SIZE, b'\xff')


def test_block_parsing():
    h, recs = proto.block_records(make_block(5, 2, 1000, 10))
    assert h.block_no == 5 and h.boot_id == 2 and len(recs) == 10 and recs[0][0] == 1000
    h, recs = proto.block_records(make_block(5, 2, 1000, 7, closed=False))   # open / power lost: count by content
    assert len(recs) == 7
    bad = bytearray(make_block(5, 2, 1000, 10))
    bad[3] ^= 1
    assert proto.block_records(bytes(bad)) == (None, [])
    assert proto.Header.unpack(b'\xff' * 48) is None


def test_timed_rows_and_guessing():
    t0 = 1792616400.0   # 23:40 local... any fixed moment
    blocks = {0: make_block(0, 1, 0, 253, int(t0 * 1000)), 1: make_block(1, 1, 253, 100),   # t0 only in the first
              2: make_block(2, 2, 0, 50),                                                    # boot 2: clock never set
              3: make_block(3, 3, 0, 20, int((t0 + 9000) * 1000))}
    rows = store.timed_rows(blocks, log=lambda *a: None)
    by = {(r['boot'], r['seq']): r for r in rows}
    assert float(by[(1, 0)]['t']) == t0 and float(by[(1, 352)]['t']) == t0 + 176
    assert by[(1, 352)]['src'] == 'flash'
    assert by[(2, 0)]['src'] == 'flash-guess' and float(by[(2, 0)]['t']) == t0 + 176.5   # right after boot 1
    assert float(by[(3, 0)]['t']) == t0 + 9000
    # nothing before it: placed backward from the next boot
    rows = store.timed_rows({2: make_block(2, 2, 0, 50), 3: make_block(3, 3, 0, 20, int(t0 * 1000))}, log=lambda *a: None)
    assert float([r for r in rows if r['boot'] == 2][0]['t']) == t0 - 25


def test_merge_and_nights(tmp_path):
    p = str(tmp_path / 'n' / 'work' / 'sensor.csv')
    rec = proto.Record.unpack(struct.pack('<hhHHHHBBH', 0, 0, 0, 0, 0, 0, 0x80, 0, 0))
    t = dt.datetime(2026, 10, 23, 23, 30).timestamp()
    a = [store.row(t + i * 0.5, 1, i, rec, 'flash-guess') for i in range(10)]
    assert store.merge(p, a) == 10
    assert store.merge(p, a) == 0                                  # again: nothing new
    b = [store.row(t + 100 + i * 0.5, 1, i, rec, 'live') for i in range(5, 15)]
    assert store.merge(p, b) == 5
    rows = store.read_csv(p)
    assert len(rows) == 15 and rows[5]['src'] == 'live'           # a known time beats a guessed one
    assert store.night_of(dt.datetime(2026, 10, 24, 3, 0).timestamp()) == dt.date(2026, 10, 23)
    assert store.night_of(dt.datetime(2026, 10, 24, 13, 0).timestamp()) == dt.date(2026, 10, 24)


# ---------- against the simulator ----------

def test_commands_and_auth(tmp_path, env):
    p, usb, ble = start_sim(tmp_path, 'seg 600 pitch 0\n', '--speed', '1')
    try:
        out = fhs(env, 'status', '--port', usb).stdout
        assert 'firmware 0.0.6' in out and 'mode shadow' in out
        assert fhs(env, 'get', '--serial-ble', ble, check=False).returncode == 1   # no key yet
        assert 'OK set angle=40' in fhs(env, 'set', 'angle', '40', '--port', usb).stdout
        assert fhs(env, 'set', 'angle', '400', '--port', usb, check=False).returncode == 1
        out = fhs(env, 'mode', 'therapy', '--port', usb, '--yes').stdout
        assert 'OK mode therapy' in out
        fhs(env, 'mode', 'shadow', '--port', usb, '--yes')
        assert 'OK zero' in fhs(env, 'zero', '--port', usb).stdout
        fhs(env, 'time', '--port', usb)
        out = fhs(env, 'status', '--port', usb).stdout
        assert 'clock: 20' in out and 'zero set: yes' in out
        # the key, as pair saves it (pair itself needs real Bluetooth)
        import json
        k = run_key(env, usb)
        json.dump({'key': k}, open(env['FHS_CONFIG'], 'w'))
        assert 'angle=40.0' in fhs(env, 'get', '--serial-ble', ble).stdout
        assert 'buzz' not in fhs(env, 'buzz-test', '60', '200', '--serial-ble', ble).stderr
    finally:
        stop(p)


def run_key(env, usb):
    code = ('import asyncio\nfrom fhs.client import Client\nfrom fhs.link import SerialLink\n'
            'async def m():\n    async with Client(SerialLink(%r)) as c:\n        print((await c.cmd("key"))[-1].split()[-1])\n'
            'asyncio.run(m())\n' % usb)
    return subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, check=True).stdout.strip()


def test_live_survives_a_dropout(tmp_path, env):
    """Bluetooth goes away for 8 s in the middle: the rows are complete anyway (catch-up from flash)"""
    scen = 'seg 10 pitch 0\nble off\nseg 8 pitch 0\nble on\nseg 600 pitch 0\n'
    p, usb, ble = start_sim(tmp_path, scen, '--speed', '1')
    try:
        for k, v in (('lie_gate', '0'), ('win_from', '0'), ('win_to', '0')):   # log from the first second
            fhs(env, 'set', k, v, '--port', usb)
        import json
        json.dump({'key': run_key(env, usb)}, open(env['FHS_CONFIG'], 'w'))
        night = tmp_path / '2026-10-23_night'
        r = fhs(env, 'live', str(night), '09:00', '--seconds', '30', '--serial-ble', ble, timeout=90)
        rows = store.read_csv(str(night / 'work' / 'sensor.csv'))
        # the morning dump: live got everything, so it reads at most the still-open block
        holes = fhs(env, 'dump', '--holes', '--serial-ble', ble, '--root', str(tmp_path), '--no-clock').stdout
    finally:
        stop(p)
    seqs = sorted(int(x['seq']) for x in rows)
    assert len(seqs) > 40, r.stdout
    assert seqs == list(range(seqs[0], seqs[-1] + 1)), 'holes: %s' % sorted(set(range(seqs[0], seqs[-1] + 1)) - set(seqs))
    assert seqs[0] <= 4                                       # from the start (seq 0 is the first half second)
    assert any(x['src'] == 'live' for x in rows)
    t = [float(x['t']) for x in sorted(rows, key=lambda x: int(x['seq']))]
    assert max(b - a for a, b in zip(t, t[1:])) < 0.6       # on laptop time, 0.5 s apart
    assert abs(t[-1] - time.time()) < 60
    assert 'link:' in r.stdout                                # the dropout was noticed
    n_listed, n_read = map(int, re.search(r'(\d+) blocks on the sensor, (\d+) to read', holes).groups())
    assert n_listed >= 1 and n_read <= 1, holes


def test_dump_after_travel(tmp_path, env):
    """two nights away: a power loss in the first (reboot, then the clock is unknown), dumped at home"""
    flash, state = str(tmp_path / 'flash.bin'), str(tmp_path / 'state.bin')
    night = 'seg 30 pitch 80\nseg 5 pitch 0 ramp\nseg 1200 roll 2\nseg 3 roll 80 ramp\nseg 1200 roll 80\n'
    t0 = int(dt.datetime(2026, 10, 23, 23, 0).timestamp())
    # the laptop set the clock before the trip; the battery dies 1500 s into the night
    run_sim(tmp_path, 't0 %d\n' % t0 + night, '--flash', flash, '--state', state, '--clock', '120', '--kill-at', '1500')
    # the sensor comes back on (no laptop: no clock) and logs the rest of the night
    run_sim(tmp_path, night, '--flash', flash, '--state', state)
    p, usb, _ = start_sim(tmp_path, 'seg 600 pitch 80\n', '--flash', flash, '--state', state, '--speed', '1', ble=False)
    try:
        # these nights had the clock until the battery died; the boot after it has none
        out = fhs(env, 'dump', '--port', usb, '--root', str(tmp_path / 'sleep'), '--no-clock').stdout
        again = fhs(env, 'dump', '--port', usb, '--root', str(tmp_path / 'sleep'), '--no-clock').stdout
    finally:
        stop(p)
    path = tmp_path / 'sleep' / '2026-10-23_night' / 'work' / 'sensor.csv'
    assert path.exists(), out
    rows = store.read_csv(str(path))
    boots = sorted({int(r['boot']) for r in rows})
    assert boots == [1, 2], out
    b1 = [r for r in rows if r['boot'] == '1']
    b2 = [r for r in rows if r['boot'] == '2']
    assert all(r['src'] == 'flash' for r in b1) and all(r['src'] == 'flash-guess' for r in b2)
    first = min(b1, key=lambda r: int(r['seq']))
    assert abs(float(first['t']) - (t0 + int(first['seq']) * 0.5)) < 0.01   # the clock set at t0
    # boot 1 lost at most its last 8 s (RAM); it logged from lying down (~31 s) to the kill at 1500 s
    s1 = sorted(int(r['seq']) for r in b1)
    assert s1 == list(range(s1[0], s1[-1] + 1)) and s1[0] <= 64 and s1[-1] >= 2 * 1500 - 18
    assert len(b2) > 2 * 2000
    assert 'nothing new' in again or '0 new' in again


def test_bench_raw(tmp_path, env):
    """bench on the simulator with the bell ringing: finds the bell's 200 Hz"""
    scen = 'bell 40 2 60 200 30\nbellwait 0 0\nseg 3 pitch 0\nseg 600 pitch 0\n'
    p, usb, _ = start_sim(tmp_path, scen, '--speed', '1', ble=False)
    try:
        time.sleep(4)
        out = fhs(env, 'bench', '--port', usb, '--seconds', '3').stdout
    finally:
        stop(p)
    zline = [ln for ln in out.splitlines() if ln.startswith('acc z')][0]
    assert 'strongest above 50 Hz: 200 Hz' in zline, out


def test_positions_guided(tmp_path, env):
    """the guided position test against the simulator (sitting up, then on the back throughout): the head axis is
    found, the test settings go in and come back out, the test's records land in tests/, not in a night"""
    import json
    scen = 'watch\nseg 14 pitch 80\nseg 2 pitch 0 ramp\nseg 300 roll 2\n'
    p, usb, _ = start_sim(tmp_path, scen, '--speed', '1', ble=False)
    try:
        fhs(env, 'axis', '2', '--port', usb)   # wrong on purpose: the test must find 0
        out = fhs(env, 'positions', '--port', usb, '--stay-connected', '--no-speak', '--hold', '1.5', '--move', '0.2',
                  '--vest', '1.5', '--settle', '1', '--root', str(tmp_path / 'sleep'), timeout=240).stdout
        after = fhs(env, 'get', '--port', usb).stdout
    finally:
        stop(p)
    assert 'head axis 0: sitting up reads pitch 8' in out, out
    assert 'start_wait=900' in after and 'lie_gate=1' in after and 'win_from=0' in after and 'mode=shadow' in after
    d = next((tmp_path / 'sleep' / 'tests').iterdir())
    res = json.load(open(d / 'results.json'))
    assert len(res['results']) == 20 and all(r['n'] > 0 for r in res['results'][1:]), out
    flat = [r for r in res['results'] if r['expect'] == 'nudge']
    assert all(abs(r['roll']) < 10 for r in flat)   # on the back throughout
    assert 'positions right' in (d / 'report.txt').read_text()
    assert not list((tmp_path / 'sleep').glob('*_night'))   # nothing in the nights
    assert json.load(open(env['FHS_CONFIG']))['dumped']   # the morning dump skips the test's blocks


def test_voice_clip(tmp_path):
    """fhs voice: speech becomes a loud 16 kHz 8-bit clip in the header the firmware includes"""
    import wave
    import numpy as np
    from fhs import voice
    t = np.arange(0, 1.0, 1 / 22050)
    x = np.sin(2 * np.pi * 800 * t) * np.where(t < 0.5, 0.05, 0.4)   # two "syllables", one quiet, one loud
    x = np.concatenate([np.zeros(4410), x, np.zeros(4410)])          # with 0.2 s of silence either side
    path = tmp_path / 'rec.wav'
    with wave.open(str(path), 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(22050)
        w.writeframes((x * 32767).astype('<i2').tobytes())
    y = voice.loud(*voice.read_wav(str(path)))
    assert abs(len(y) / voice.RATE - 1.0) < 0.03                       # silence trimmed, resampled
    quiet, loud_ = np.abs(y[:len(y) // 2]).mean(), np.abs(y[len(y) // 2:]).mean()
    assert quiet > 0.5 * loud_ and np.abs(y).max() <= 1               # the syllables evened out
    h = tmp_path / 'prompts.h'
    voice.write_prompts([('rec', y)], str(h))
    assert 'PROMPT_0[%d]' % len(y) in h.read_text()


