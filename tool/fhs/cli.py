"""fhs: the laptop side of the forehead sensor. fhs <command> --help for details.

Connecting: --usb (the sensor on a USB cable; --port PORT to name the port) or --ble (the paired sensor;
--address ADDR for another one). With neither: USB if a sensor is plugged in, else Bluetooth."""
import argparse
import asyncio
import datetime as dt
import hashlib
import json
import os
import sys
import time

from . import positions, proto, store, voice
from .client import Client, DeviceError
from .link import BleLink, LinkLost, SerialLink, find_usb, scan

SLEEP_ROOT = os.path.expanduser('~/sleep-data')


def log(*a):
    print(dt.datetime.now().strftime('%H:%M:%S'), 'sensor', *a, flush=True)


def make_link(args, cfg):
    if getattr(args, 'serial_ble', None):   # tests: the simulator's Bluetooth pty (needs the key, like Bluetooth)
        return SerialLink(args.serial_ble, trusted=False)
    if args.usb or args.port:
        port = args.port or find_usb()
        if not port:
            raise DeviceError('no sensor on USB')
        return SerialLink(port)
    if not args.ble and not args.address:   # nothing asked: USB if it's plugged in, else Bluetooth
        port = find_usb()
        if port:
            return SerialLink(port)
    return BleLink(args.address or cfg.get('ble', {}).get('address'))


def client(args, cfg, logf=log):
    return Client(make_link(args, cfg), key=cfg.get('key'), log=logf)


def device_id(key):
    return hashlib.sha1(key.encode()).hexdigest()[:8]


def until_ts(hhmm, now=None):
    """the next HH:MM from now (today, or tomorrow if it has passed)"""
    now = now or dt.datetime.now()
    h, m = map(int, hhmm.split(':'))
    end = now.replace(hour=h, minute=m, second=0, microsecond=0)
    if end <= now:
        end += dt.timedelta(days=1)
    return end.timestamp()


# ---------- small commands ----------

async def cmd_status(args, cfg):
    async with client(args, cfg) as c:
        st = await c.get_status()
        p = await c.get_params()
    t0 = int(st.get('t0', 0)) / 1000
    print('firmware %s, boot %s, mode %s' % (st.get('fw'), st.get('boot'), st.get('mode')))
    print('battery %s mV, usb %s, charging %s%s' % (st.get('batt'), st.get('usb'), st.get('chg'),
                                                  ', LOW: logging stopped' if st.get('cutoff') == '1' else ''))
    print('clock: %s' % (dt.datetime.fromtimestamp(t0 + int(st['seq']) * 0.5).strftime('%Y-%m-%d %H:%M:%S')
                         if st.get('clock') == '1' else 'not set (fhs time)'))
    print('log: %s of %s blocks used (%.1f h), logging now: %s' % (
        st.get('blocks'), st.get('sectors'), int(st.get('blocks', 0)) * proto.RECS_PER_BLOCK * 0.5 / 3600,
        'yes' if st.get('session') == '1' else 'no'))
    print('head: roll %s, pitch %s; zero set: %s (axis %s); flat for %s s, nudge step %s' % (
        st.get('roll'), st.get('pitch'), 'yes' if st.get('zero_set') == '1' else 'no', st.get('axis'),
        st.get('flat_s'), st.get('step')))
    if st.get('hw'):
        print('hardware: ' + st['hw'])
    if st.get('err', '0') != '0':
        print('flash errors: ' + st['err'])
    print('settings: ' + ' '.join('%s=%s' % kv for kv in p.items()))


async def cmd_simple(args, cfg, line, show=True):
    async with client(args, cfg) as c:
        out = await c.cmd(line)
    if show:
        for ln in out:
            print(ln)


async def cmd_mode(args, cfg):
    if args.mode == 'therapy' and not args.yes:
        if input('Therapy mode: the sensor nudges (vibration, then sound) when the head is flat. Continue? [y/N] ').strip().lower() != 'y':
            print('left as it is')
            return
    await cmd_simple(args, cfg, 'mode ' + args.mode)


async def cmd_time(args, cfg):
    async with client(args, cfg) as c:
        boot, t0 = await c.set_time()
    print('clock set (boot %d)' % boot)


async def cmd_erase(args, cfg):
    if not args.yes:
        print('this wipes the sensor\'s log; run "fhs dump" first, then: fhs erase --yes')
        return
    await cmd_simple(args, cfg, 'erase yes')


async def cmd_pair(args, cfg):
    port = args.port or find_usb()
    if not port:
        raise DeviceError('plug the sensor in over USB to pair (the key is only given over USB)')
    async with Client(SerialLink(port), log=log) as c:
        key = (await c.cmd('key'))[-1].split()[-1]
        await c.set_time()
    print('key read over USB')
    found = await scan(args.timeout)
    if args.address:
        found = [f for f in found if f[0].lower() == args.address.lower()] or [(args.address, None, 0)]
    if not found:
        raise DeviceError('no sensor seen over Bluetooth: is the laptop\'s Bluetooth on?')
    if len(found) > 1:
        print('more than one sensor in range:', ', '.join('%s %s (%d dBm)' % f for f in found),
              '- taking the strongest; use --address to pick')
    addr, name, rssi = found[0]
    cfg.update(key=key, ble=dict(address=addr, name=name), device=device_id(key))
    store.save_config(cfg)
    print('paired with %s %s (%s dBm); saved in %s' % (name, addr, rssi, store.CONFIG))


async def cmd_scan(args, cfg):
    for addr, name, rssi in await scan(args.timeout):
        print('%s  %s  %d dBm' % (addr, name, rssi))


async def cmd_watch(args, cfg):
    """print the records as they come (for the bench test)"""
    async with client(args, cfg) as c:
        await c.cmd('live on')
        print('time      roll  pitch  move   buzz    mic  snore  step would var flags')
        end = time.monotonic() + args.seconds if args.seconds else None
        while end is None or time.monotonic() < end:
            item = await c.records.get()
            if item is None:
                raise LinkLost()
            _, _, r = item
            print('%s %6.1f %6.1f %5.1f %6.1f %6d %6d %5d %5d %3d %s' % (
                dt.datetime.now().strftime('%H:%M:%S'), r.roll, r.pitch, r.move, r.buzz, r.mic, r.snore, r.motor,
                r.would, r.variant, ','.join(r.flag_names())), flush=True)


async def cmd_bench(args, cfg):
    """raw samples over USB for a few seconds, then the spectrum: shows the bell's buzz frequency and strength"""
    import numpy as np
    args.ble, args.address, args.serial_ble, args.usb = False, None, None, True
    what = 'mic' if args.mic else 'imu'
    chunks = []
    async with client(args, cfg) as c:
        watch = 'pmu=' in (await c.get_status()).get('hw', '')   # the T-Watch: BMA423, 800 Hz, 12 bits, no gyro
        await c.cmd('raw ' + what)
        end = time.monotonic() + args.seconds
        print('recording %s for %d s ...' % (what, args.seconds), flush=True)
        while time.monotonic() < end:
            ln = await c.next_event('M' if args.mic else 'X', 2)
            if ln:
                chunks.append(bytes.fromhex(ln.split()[2]))
        await c.cmd('raw off')
    raw = np.frombuffer(b''.join(chunks), dtype='<i2')
    if args.mic:
        x, fs = raw.astype(float), 16000.0
        cols = {'mic': x - x.mean()}
    else:
        s = raw[: len(raw) // 6 * 6].reshape(-1, 6).astype(float)
        fs = 800.0 if watch else 833.0
        acc, gyr = s[:, :3] * (1000 / 1024 if watch else 0.122), s[:, 3:] * 0.0175   # mg, deg/s
        g = acc.mean(0)
        print('samples %d (%.1f s at %.0f Hz); gravity %.0f %.0f %.0f mg (|g| = %.0f)' % (
            len(s), len(s) / fs, fs, g[0], g[1], g[2], np.linalg.norm(g)))
        if not watch:
            print('gyro mean %.2f %.2f %.2f deg/s, RMS about the mean %.2f deg/s' % (
                *gyr.mean(0), np.sqrt(((gyr - gyr.mean(0)) ** 2).sum(1).mean())))
        cols = {'acc x': acc[:, 0] - g[0], 'acc y': acc[:, 1] - g[1], 'acc z': acc[:, 2] - g[2]}
    if not len(raw):
        raise DeviceError('no raw samples came')
    lo = 50 if not args.mic else 60
    for name, x in cols.items():
        n = len(x)
        w = np.hanning(n)
        spec = np.abs(np.fft.rfft(x * w)) * 2 / w.sum()     # amplitude of a sine at each frequency
        f = np.fft.rfftfreq(n, 1 / fs)
        band = f >= lo
        top = np.argsort(spec[band])[::-1]
        peaks, used = [], []
        for i in top:
            fi = f[band][i]
            if all(abs(fi - u) > 10 for u in used):
                peaks.append('%.0f Hz %.1f' % (fi, spec[band][i]))
                used.append(fi)
            if len(peaks) == 3:
                break
        X = np.fft.rfft(x)
        hp = np.sqrt(2 * (np.abs(X[f >= 80]) ** 2).sum()) / n   # what the firmware's buzz (or snore) number sees
        print('%-6s above 80 Hz: RMS %.1f%s; strongest above %d Hz: %s' % (name, hp, ' mg' if not args.mic else '', lo,
                                                                          ', '.join(peaks)))
    if args.out:
        np.savetxt(args.out, raw.reshape(-1, 1 if args.mic else 6), fmt='%d', delimiter=',',
                   header='mic' if args.mic else 'ax,ay,az,gx,gy,gz (raw: %s)' % (
                       '0.977 mg per unit, 800 Hz, no gyro' if watch else '0.122 mg, 17.5 mdps per unit, 833 Hz'))
        print('raw samples saved to ' + args.out)


# ---------- the night: live recording over USB ----------

async def cmd_live(args, cfg):
    work = os.path.join(os.path.abspath(args.night), 'work')
    path = os.path.join(work, 'sensor.csv')
    end = time.time() + args.seconds if args.seconds else until_ts(args.until)
    stop = asyncio.Event()   # a recording script can stop us with TERM: finish cleanly
    import signal
    for sig in (signal.SIGTERM, signal.SIGINT):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)
    if not cfg.get('key') and not (args.usb or args.port) and not args.serial_ble:
        log('not paired (fhs pair): no live sensor data tonight')
        return
    app = store.Appender(path)
    seen = {(int(r['boot']), int(r['seq'])) for r in store.read_csv(path)}
    t0s = {}             # boot -> unix s of its record 0, from our own clock setting
    backoff, got, last_note, fails, last_fail_log = 2, 0, 0.0, 0, 0.0
    log('live until %s into %s (%d rows there already)' % (dt.datetime.fromtimestamp(end).strftime('%H:%M'), path, len(seen)))
    while time.time() < end and not stop.is_set():
        try:
            async with client(args, cfg) as c:
                boot, t0s[boot] = await c.set_time()
                st = await c.get_status()
                log('connected (boot %d, battery %s mV, logging %s, %s)' % (
                    boot, st.get('batt'), 'on' if st.get('session') == '1' else 'off', st.get('mode')))
                backoff, fails = 2, 0
                # the sensor rebooted since our last record: get the rest of the old boot from its flash first
                mine = sorted(k for k in seen if k[0] < boot)
                if mine and not any(k[0] == boot for k in seen):
                    old = mine[-1][0]
                    for b in await c.blocks():
                        if b['boot'] == old and b['t0_ms']:
                            t0s[old] = b['t0_ms'] / 1000
                    if old in t0s:
                        await c.cmd('since %d %d' % (old, mine[-1][1] + 1))
                        await c.next_event('C', 120)
                hi = max((s for b, s in seen if b == boot), default=-1)
                await c.cmd('since %d %d live' % (boot, hi + 1))
                catching = True
                last_rec = time.monotonic()
                while time.time() < end and not stop.is_set():
                    try:
                        item = await asyncio.wait_for(c.records.get(), 2)
                    except asyncio.TimeoutError:
                        item = False
                    if item is None:
                        raise LinkLost()
                    if item is False:
                        if time.monotonic() - last_rec > 10:   # silent link (records come every 0.5 s): ask again
                            await c.cmd('since %d %d live' % (boot, hi + 1))
                            last_rec = time.monotonic()
                        continue
                    last_rec = time.monotonic()
                    b, s, r = item
                    if (b, s) in seen or b not in t0s:
                        continue
                    seen.add((b, s))
                    app.add(store.row(t0s[b] + s * proto.WINDOW_S, b, s, r, 'live'))
                    got += 1
                    if b == boot:
                        if s > hi + 1 and not catching:   # records dropped on the way: fetch the hole from flash
                            await c.cmd('since %d %d live' % (boot, hi + 1))
                            catching = True
                        hi = max(hi, s)
                    ev = None
                    while not c.events.empty():
                        ev = c.events.get_nowait()
                        if ev and ev.startswith('C '):
                            catching = False
                    if time.time() - last_note > 600:
                        last_note = time.time()
                        st = await c.get_status()
                        log('%d rows; battery %s mV; logging %s; dropped %s' % (got, st.get('batt'), st.get('session'), st.get('dropped')))
                await c.cmd('live off')
        except Exception as e:   # Bluetooth stacks raise all kinds; the night goes on regardless
            app.flush()
            if time.time() >= end or stop.is_set():
                break
            fails += 1
            if fails == 1 or time.time() - last_fail_log > 1800:   # not a line a minute all night
                last_fail_log = time.time()
                log('link: %s: %s; trying again every %d s' % (type(e).__name__, e, min(backoff * 2, 60)))
            try:
                await asyncio.wait_for(stop.wait(), min(backoff, max(0, end - time.time())))
            except asyncio.TimeoutError:
                pass
            backoff = min(backoff * 2, 60)
    app.flush()
    log('live stopped, %d rows this run' % got)


# ---------- after travel: the nights from the sensor's flash ----------

def runs(nums):
    """[(first, count)] of consecutive numbers"""
    out = []
    for x in sorted(nums):
        if out and x == out[-1][0] + out[-1][1]:
            out[-1][1] += 1
        else:
            out.append([x, 1])
    return [tuple(r) for r in out]


_keys = {}


def has_hole(b, listed, root):
    """a listed block has records missing from its night's sensor.csv (or its time is unknown)"""
    t0 = max((x['t0_ms'] for x in listed if x['boot'] == b['boot']), default=0) / 1000   # the boot's newest clock
    if not t0 or not b['n']:
        return bool(b['n'])
    keys = set()
    for seq in (b['first_seq'], b['first_seq'] + b['n'] - 1):
        path = store.night_csv(root, store.night_of(t0 + seq * proto.WINDOW_S))
        if path not in _keys:
            _keys[path] = {(int(r['boot']), int(r['seq'])) for r in store.read_csv(path)}
        keys |= _keys[path]
    return any((b['boot'], b['first_seq'] + i) not in keys for i in range(b['n']))


def store_nights(dev, blocks, hint, root):
    """the nights the new blocks belong to, from the block archive into their sensor.csv files. Returns [(evening date, rows, new rows)]."""
    # the boots in the new blocks, with their neighbours (for boots whose clock was never set)
    new_boots = {proto.Header.unpack(b).boot_id for b in blocks.values() if proto.Header.unpack(b)}
    arch = store.load_blocks(dev)
    if new_boots:
        lo, hi = min(new_boots) - 1, max(new_boots) + 1
        arch = {bn: b for bn, b in arch.items() if proto.Header.unpack(b) and lo <= proto.Header.unpack(b).boot_id <= hi}
    else:
        arch = {}
    rows = store.timed_rows(arch, hint, log=print)
    if not rows:
        print('nothing new')
    out = []
    for day, rs in sorted(store.split_nights(rows).items()):
        path = store.night_csv(root, day)
        n = store.merge(path, rs)
        guess = sum(r['src'] == 'flash-guess' for r in rs)
        print('%s night: %d rows (%d new), %.1f h%s -> %s' % (day, len(rs), n, len(rs) * 0.5 / 3600,
                                                              ', time guessed' if guess else '', path))
        out.append((day, len(rs), n))
    return out


async def cmd_dump(args, cfg):
    root = os.path.abspath(args.root or cfg.get('root') or SLEEP_ROOT)
    if args.quiet_unpaired and not cfg.get('key') and not (args.usb or args.port) and not find_usb():
        return   # no sensor paired yet
    async with client(args, cfg) as c:
        st = await c.get_status()
        if c.link.trusted:
            key = (await c.cmd('key'))[-1].split()[-1]
        else:
            key = cfg.get('key')
        dev = device_id(key)
        hint = {}
        if st.get('clock') == '1':
            hint[int(st['boot'])] = int(st['t0']) / 1000
            # the watch counts time with its own oscillator, which runs a little off while it sleeps: how far by now
            off = int(st['t0']) / 1000 + (int(st['seq']) + 0.5) * proto.WINDOW_S - time.time()
            print('sensor clock %+.1f s from the laptop\'s' % off, flush=True)
        if not args.no_clock:
            # set it now: the boot running now gets its time from this, so the night is timed from this morning
            # rather than from the power-up (maybe days ago), and the clock chip stays right too
            boot, t0 = await c.set_time()
            hint[boot] = t0
        listed = await c.blocks()
        done = cfg.get('dumped', {}).get(dev, 0)
        start = 0 if args.all else done
        todo = [b for b in listed if b['block_no'] >= start]
        if args.holes:   # only blocks with records the nights' sensor.csv files don't have yet (live got the rest)
            todo = [b for b in todo if has_hole(b, listed, root)]
        print('%d blocks on the sensor, %d to read' % (len(listed), len(todo)), flush=True)
        blocks = {}
        for first, n in runs([b['block_no'] for b in todo]):
            blocks.update(await c.dump(first, n))
    store.save_blocks(dev, blocks)
    store_nights(dev, blocks, hint, root)
    closed = [b['block_no'] for b in listed if not b['open'] and b['block_no'] in blocks]
    if closed:
        cfg.setdefault('dumped', {})[dev] = max(max(closed) + 1, done)
        store.save_config(cfg)


# ---------- main ----------

# ---------- the guided position test (positions.py) ----------

async def restore_settings(c, before):
    for k in positions.TEST_SETTINGS:
        if k in before:
            await c.cmd('set %s %s' % (k, before[k]))
    await c.cmd('mode ' + before.get('mode', 'shadow'))


async def cmd_positions(args, cfg):
    """the laptop speaks one position after the other; the watch logs them unplugged; then the report"""
    import json
    speak = not args.no_speak
    root = os.path.abspath(args.root or cfg.get('root') or SLEEP_ROOT)
    args.ble, args.address, args.serial_ble = False, None, None
    args.usb = True
    if args.restore:   # a test that was cut short: put the night's settings back
        before = json.load(open(os.path.join(args.restore, 'settings_before.json')))
        async with client(args, cfg) as c:
            await restore_settings(c, before)
        print('settings put back from ' + args.restore)
        return
    out = os.path.join(root, 'tests', dt.datetime.now().strftime('%Y-%m-%d_%H%M') +
                       ('_therapy' if args.therapy else '_positions'))
    os.makedirs(out, exist_ok=True)
    # 1. plugged in: the nights on the watch first (so no test record ever lands in a night), then the test settings
    await cmd_dump(argparse.Namespace(root=root, quiet_unpaired=False, usb=True, port=args.port, ble=False, address=None,
                                      serial_ble=None, no_clock=False, all=False, holes=False), cfg)
    async with client(args, cfg) as c:
        before = await c.get_params()
        json.dump(before, open(os.path.join(out, 'settings_before.json'), 'w'), indent=1)
        print('if the test is cut short: fhs positions --restore %s' % out)
        key = (await c.cmd('key'))[-1].split()[-1]
        await c.set_time()
        if args.therapy and (await c.get_status()).get('zero_set') != '1':
            raise DeviceError('the zero is not set yet: run "fhs positions" (without --therapy) first')
        if not args.keep_axis:   # which board axis points to the top of the head: the one that reads upright
            positions.say(('Put on the vest and the eye mask. ' if args.therapy else 'Leave the vest off for now. ') +
                          'Tape the watch on your forehead, sit up straight and keep still.', speak)
            await asyncio.sleep(args.settle)
            best = None
            for ax in range(4):
                await c.cmd('axis %d' % ax)
                await asyncio.sleep(1.2)
                pitch = float((await c.get_status()).get('pitch', 0))
                if best is None or pitch > best[1]:
                    best = (ax, pitch)
            await c.cmd('axis %d' % best[0])
            print('head axis %d: sitting up reads pitch %.0f' % best)
            if best[1] < 60:
                print('!! no axis reads sitting up as upright: is the watch on the forehead?')
        for k, v in positions.TEST_SETTINGS.items():
            await c.cmd('set %s %s' % (k, v))
        await c.cmd('mode ' + ('therapy' if args.therapy else 'shadow'))
    t_start = time.time()
    # 2. unplugged, lying down: one position after the other
    positions.say('Now unplug the watch and lie down ' + ('on your left side' if args.therapy else
                  'flat on your back, without the vest') + '. The first position comes in a moment.', speak)
    if not args.stay_connected:
        end = time.monotonic() + 300
        while find_usb() and time.monotonic() < end:
            await asyncio.sleep(0.5)
        if find_usb():
            raise DeviceError('still plugged in after 5 min: run fhs positions --restore %s, then start again' % out)
    await asyncio.sleep(args.settle)
    segs = []
    for text, expect, secs in positions.plan(args.therapy, args.hold, args.vest):
        positions.say(text, speak)
        await asyncio.sleep(args.move)   # time to get there
        t0 = time.time()
        await asyncio.sleep(secs)
        segs.append(dict(text=text, expect=expect, start=t0, end=time.time()))
        json.dump(segs, open(os.path.join(out, 'positions.json'), 'w'), indent=1)
    positions.say('Done. Thank you. Plug the watch back in.', speak)
    # 3. plugged in again: the test's blocks, the settings back, the report
    if not args.stay_connected:
        end = time.monotonic() + 1800
        while not find_usb() and time.monotonic() < end:
            await asyncio.sleep(1)
        if not find_usb():
            raise DeviceError('not plugged back in: when it is, run fhs positions --restore %s' % out)
        await asyncio.sleep(3)
    async with client(args, cfg) as c:
        await asyncio.sleep(1.5)   # on USB the session ends and its block closes
        st = await c.get_status()
        hint = {int(st['boot']): int(st['t0']) / 1000} if st.get('clock') == '1' else {}
        t_end = time.time()

        def span(b):   # the block's time span (unix s), from its boot's clock
            t0 = b['t0_ms'] / 1000
            return t0 + b['first_seq'] * proto.WINDOW_S, t0 + (b['first_seq'] + b['n']) * proto.WINDOW_S
        mine = [b['block_no'] for b in await c.blocks()
                if b['t0_ms'] and span(b)[0] <= t_end and (b['open'] or span(b)[1] >= t_start)]
        blocks = {}
        for first, n in runs(mine):
            blocks.update(await c.dump(first, n))
        await restore_settings(c, before)
        params = await c.get_params()
    dev = device_id(key)
    store.save_blocks(dev, blocks)
    if mine:   # the morning dump skips the test's blocks
        cfg.setdefault('dumped', {})[dev] = max(max(mine) + 1, cfg.get('dumped', {}).get(dev, 0))
        store.save_config(cfg)
    rows = [r for r in store.timed_rows(blocks, hint, log=print) if t_start <= float(r['t']) <= t_end]
    import csv
    with open(os.path.join(out, 'sensor.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, store.COLUMNS)
        w.writeheader()
        w.writerows(rows)
    angle, hyst = float(params.get('angle', 60)), float(params.get('hyst', 5))
    res = positions.analyze(rows, segs, angle, hyst, args.therapy)
    advice = positions.angle_advice(res, angle, hyst)
    text = positions.report(res, advice, args.therapy)
    json.dump(dict(angle=angle, hyst=hyst, results=res, advice=advice), open(os.path.join(out, 'results.json'), 'w'),
              indent=1)
    open(os.path.join(out, 'report.txt'), 'w').write(text + '\n')
    print(text)
    print('saved in ' + out)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='fhs', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    con = argparse.ArgumentParser(add_help=False)
    con.add_argument('--usb', action='store_true', help='over USB')
    con.add_argument('--port', default=None, help='over USB, this port (default: found by itself)')
    con.add_argument('--ble', action='store_true', help='over Bluetooth')
    con.add_argument('--address', default=None, help='over Bluetooth, this sensor (default: the paired one)')
    con.add_argument('--serial-ble', default=None, help=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status', parents=[con], help='battery, clock, log, head angle, settings')
    sub.add_parser('get', parents=[con], help='all settings')
    p = sub.add_parser('set', parents=[con], help='change a setting (fhs get lists them)')
    p.add_argument('key')
    p.add_argument('value')
    p = sub.add_parser('mode', parents=[con], help='shadow (nudges only logged, default) or therapy (it nudges)')
    p.add_argument('mode', choices=['shadow', 'therapy'])
    p.add_argument('--yes', action='store_true')
    p = sub.add_parser('axis', parents=[con], help='board axis pointing to the top of the head: 0 +X, 1 -X, 2 +Y, 3 -Y')
    p.add_argument('n', choices=['0', '1', '2', '3'])
    sub.add_parser('zero', parents=[con], help='lying flat on the back, still: this is 0 deg')
    p = sub.add_parser('buzz-test', parents=[con], help='run our motor (works in shadow mode too)')
    p.add_argument('level', nargs='?', type=int, default=50)
    p.add_argument('ms', nargs='?', type=int, default=1000)
    p = sub.add_parser('nudge-test', parents=[con],
                       help='play one nudge step: 1-2 vibration, 3-4 vibration + sound (works in shadow mode too)')
    p.add_argument('step', type=int, choices=[1, 2, 3, 4])
    p.add_argument('variant', nargs='?', type=int, default=0, help='0-15 (default 0)')
    p.add_argument('ms', nargs='?', type=int, default=3000)
    sub.add_parser('motor-diag', parents=[con], help='the vibration driver tests its motor: there, open or shorted')
    sub.add_parser('time', parents=[con], help='set the sensor clock from the laptop')
    sub.add_parser('session', parents=[con], help='restart the start wait (therapy)')
    p = sub.add_parser('erase', parents=[con], help='wipe the log (after a dump)')
    p.add_argument('--yes', action='store_true')
    sub.add_parser('reboot', parents=[con])
    p = sub.add_parser('pair', help='learn the key over USB and the Bluetooth address')
    p.add_argument('--port', default=None, help='USB port (default: found by itself)')
    p.add_argument('--address', default=None, help='this Bluetooth address (default: the strongest sensor in range)')
    p.add_argument('--timeout', type=float, default=8)
    p = sub.add_parser('scan', help='sensors in Bluetooth range')
    p.add_argument('--timeout', type=float, default=8)
    p = sub.add_parser('watch', parents=[con], help='print every record as it comes')
    p.add_argument('--seconds', type=float, default=0)
    p = sub.add_parser('bench', parents=[con], help='raw samples + spectrum over USB (bell buzz frequency)')
    p.add_argument('--seconds', type=int, default=10)
    p.add_argument('--mic', action='store_true', help='the mic instead of the motion sensor')
    p.add_argument('--out', default=None, help='save the raw samples (CSV)')
    p = sub.add_parser('live', parents=[con], help='record a night into <night>/work/sensor.csv ')
    p.add_argument('night')
    p.add_argument('until', help='HH:MM')
    p.add_argument('--seconds', type=float, default=0, help=argparse.SUPPRESS)   # tests: run this long instead
    p = sub.add_parser('positions', help='the guided position test: the laptop speaks the positions, the watch logs them')
    p.add_argument('--therapy', action='store_true', help='therapy mode: feel the nudges and turn (default: silent)')
    p.add_argument('--hold', type=float, default=30, help='seconds in each position (default 30)')
    p.add_argument('--move', type=float, default=5, help='seconds to get into each position (default 5)')
    p.add_argument('--vest', type=float, default=60, help='seconds to put the vest on after the zero (default 60)')
    p.add_argument('--settle', type=float, default=6, help=argparse.SUPPRESS)
    p.add_argument('--keep-axis', action='store_true', help='don\'t look for the head axis again')
    p.add_argument('--no-speak', action='store_true', help='print the positions only')
    p.add_argument('--stay-connected', action='store_true', help=argparse.SUPPRESS)   # tests: the simulator's pty
    p.add_argument('--restore', default=None, metavar='DIR', help='put back the settings a cut-short test changed')
    p.add_argument('--port', default=None, help='USB port (default: found by itself)')
    p.add_argument('--root', default=None, help='data folder (default %s); the test goes into tests/' % SLEEP_ROOT)
    p = sub.add_parser('voice', help='make the spoken zero\'s phrases (prompts.h); flash it after')
    p.add_argument('--slow', type=float, default=1.0, help='the Piper voice slower (1.2) or faster (0.8)')
    p.add_argument('--dry', action='store_true', help='only play it, don\'t save')
    p.add_argument('--no-play', action='store_true', help='don\'t play it on the laptop')
    p = sub.add_parser('dump', parents=[con], help='read the nights from the sensor\'s flash into the data folder')
    p.add_argument('--root', default=None, help='data folder (default %s)' % SLEEP_ROOT)
    p.add_argument('--all', action='store_true', help='read every block, not just the new ones')
    p.add_argument('--no-clock', action='store_true', help='don\'t set the sensor clock')
    p.add_argument('--holes', action='store_true', help='only blocks with records the nights don\'t have yet (after live)')
    p.add_argument('--quiet-unpaired', action='store_true', help=argparse.SUPPRESS)   # scripts
    args = ap.parse_args(argv)
    if args.cmd == 'voice':
        return voice.main(args)
    cfg = store.load_config()
    fn = {'status': cmd_status, 'mode': cmd_mode, 'time': cmd_time, 'erase': cmd_erase, 'pair': cmd_pair,
          'scan': cmd_scan, 'watch': cmd_watch, 'bench': cmd_bench, 'live': cmd_live, 'dump': cmd_dump,
          'positions': cmd_positions}.get(args.cmd)
    if fn:
        coro = fn(args, cfg)
    else:
        line = {'get': 'get', 'set': 'set %s %s' % (getattr(args, 'key', ''), getattr(args, 'value', '')),
                'axis': 'axis %s' % getattr(args, 'n', ''), 'zero': 'zero',
                'buzz-test': 'buzz %s %s' % (getattr(args, 'level', ''), getattr(args, 'ms', '')),
                'nudge-test': 'nudge %s %s %s' % (getattr(args, 'step', ''), getattr(args, 'variant', ''),
                                                  getattr(args, 'ms', '')),
                'motor-diag': 'diag', 'session': 'session', 'reboot': 'reboot'}[args.cmd]
        coro = cmd_simple(args, cfg, line)
    try:
        asyncio.run(coro)
    except (DeviceError, LinkLost, OSError) as e:
        print('fhs: %s' % (e or 'link lost'), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
