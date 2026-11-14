"""Records on disk: laptop config, the raw block archive, and the per-night sensor.csv files in the data folder.

sensor.csv has one row per 0.5 s window, on laptop time:
  t        unix seconds of the window start
  time     local HH:MM:SS.s
  boot,seq the device's boot id and record number (unique together; used to merge without duplicates)
  roll,pitch  degrees (0,0 = flat on the back; roll + = left ear down; pitch + = head up)
  move     deg/s    buzz  mg    mic, snore  raw PCM RMS
  motor    the nudge step that ran in the window: 0 none, 1-2 vibration, 3-4 vibration + sound, 5 a test
  would    the step the therapy rules asked for (what would have run, in shadow mode)
  variant  0-15: the vibration pattern + sound that step played (they change every step, against getting used to it)
  flags    proto.F_* bits;  src  live | flash | flash-guess (time guessed: the clock wasn't set in that boot)
"""
import csv
import datetime as dt
import fcntl
import json
import os
import time
from contextlib import contextmanager

from . import proto

CONFIG = os.path.expanduser(os.environ.get('FHS_CONFIG', '~/.config/forehead-sensor/config.json'))
DATA = os.path.expanduser(os.environ.get('FHS_DATA', '~/.local/share/forehead-sensor'))
COLUMNS = ['t', 'time', 'boot', 'seq', 'roll', 'pitch', 'move', 'buzz', 'mic', 'snore', 'motor', 'would', 'variant',
           'flags', 'src']


def load_config():
    try:
        with open(CONFIG) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_config(c):
    os.makedirs(os.path.dirname(CONFIG), exist_ok=True)
    tmp = CONFIG + '.tmp'
    with open(os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), 'w') as f:   # holds the Bluetooth key
        json.dump(c, f, indent=1)
    os.replace(tmp, CONFIG)


def row(t, boot, seq, r: proto.Record, src):
    return dict(t='%.1f' % t, time=dt.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:10], boot=boot, seq=seq,
                roll='%.2f' % r.roll, pitch='%.2f' % r.pitch, move='%.1f' % r.move, buzz='%.1f' % r.buzz,
                mic=r.mic, snore=r.snore, motor=r.motor, would=r.would, variant=r.variant, flags=r.flags, src=src)


def night_of(t):
    """the evening date a moment belongs to: noon to noon"""
    return (dt.datetime.fromtimestamp(t) - dt.timedelta(hours=12)).date()


@contextmanager
def locked(path):
    """an exclusive lock next to path, so live and dump never write the same file at once"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + '.lock', 'w') as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def last_key(path):
    """(boot, seq) of the newest row of a sensor.csv, or None"""
    rows = read_csv(path)
    return max(((int(r['boot']), int(r['seq'])) for r in rows), default=None)


def merge(path, rows):
    """add rows to a sensor.csv: one row per (boot, seq), sorted by time. Returns how many were new."""
    with locked(path):
        old = {(int(r['boot']), int(r['seq'])): r for r in read_csv(path)}
        n = 0
        for r in rows:
            k = (int(r['boot']), int(r['seq']))
            # a row with a known time beats a guessed one; otherwise the first one stays
            if k not in old or (old[k]['src'] == 'flash-guess' and r['src'] != 'flash-guess'):
                n += k not in old
                old[k] = r
        tmp = path + '.tmp'
        with open(tmp, 'w', newline='') as f:
            w = csv.DictWriter(f, COLUMNS)
            w.writeheader()
            w.writerows(sorted(old.values(), key=lambda r: (float(r['t']), int(r['boot']), int(r['seq']))))
        os.replace(tmp, path)
    return n


class Appender:
    """appends live rows to a sensor.csv (header written once), flushing every few seconds"""

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.buf, self.last_flush = [], time.monotonic()

    def add(self, r):
        self.buf.append(r)
        if time.monotonic() - self.last_flush > 5:
            self.flush()

    def flush(self):
        self.last_flush = time.monotonic()
        if not self.buf:
            return
        with locked(self.path):
            new = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
            with open(self.path, 'a', newline='') as f:
                w = csv.DictWriter(f, COLUMNS)
                if new:
                    w.writeheader()
                w.writerows(self.buf)
        self.buf = []


# ---------- the raw block archive and turning blocks into timed rows ----------

def archive_dir(device):
    return os.path.join(DATA, device.replace(':', ''), 'blocks')


def save_blocks(device, blocks):
    d = archive_dir(device)
    os.makedirs(d, exist_ok=True)
    for bn, b in blocks.items():
        with open(os.path.join(d, '%08d.bin' % bn), 'wb') as f:
            f.write(b)


def load_blocks(device):
    d = archive_dir(device)
    out = {}
    for name in sorted(os.listdir(d)) if os.path.isdir(d) else []:
        if name.endswith('.bin'):
            with open(os.path.join(d, name), 'rb') as f:
                out[int(name[:-4])] = f.read()
    return out


def timed_rows(blocks, t0_hint=None, log=print):
    """all records of the blocks as rows on unix time. A boot's time comes from its blocks' clock (the newest
    one set wins), or t0_hint {boot: unix s} (from the device's live status). A boot whose clock was never set
    is placed right after the boot before it (or right before the next one): src 'flash-guess'."""
    by_boot = {}
    for bn in sorted(blocks):
        h, recs = proto.block_records(blocks[bn])
        if h is None:
            continue
        b = by_boot.setdefault(h.boot_id, dict(t0=None, recs={}))
        if h.t_seq0_ms:
            b['t0'] = h.t_seq0_ms / 1000   # later blocks (by block_no) override: the newest clock setting
        for seq, r in recs:
            b['recs'][seq] = r
    for boot, t0 in (t0_hint or {}).items():
        if boot in by_boot and t0:
            by_boot[boot]['t0'] = t0
    boots = sorted(by_boot)
    t0s, guessed = {}, set()
    for b in boots:
        if by_boot[b]['t0'] and by_boot[b]['recs']:
            t0s[b] = by_boot[b]['t0']
    for i, b in enumerate(boots):   # chain unknown boots forward from the boot before
        if b in t0s or not by_boot[b]['recs']:
            continue
        prev = boots[i - 1] if i else None
        if prev in t0s:
            end = t0s[prev] + (max(by_boot[prev]['recs']) + 1) * proto.WINDOW_S
            t0s[b] = end - min(by_boot[b]['recs']) * proto.WINDOW_S
            guessed.add(b)
    for i in range(len(boots) - 1, -1, -1):   # ... or backward from the boot after
        b = boots[i]
        if b in t0s or not by_boot[b]['recs']:
            continue
        nxt = boots[i + 1] if i + 1 < len(boots) else None
        if nxt in t0s:
            start = t0s[nxt] + min(by_boot[nxt]['recs']) * proto.WINDOW_S
            t0s[b] = start - (max(by_boot[b]['recs']) + 1) * proto.WINDOW_S
            guessed.add(b)
    rows = []
    for b in boots:
        if b not in t0s:
            if by_boot[b]['recs']:
                log(f'boot {b}: {len(by_boot[b]["recs"])} records skipped: its clock was never set and no boot next to it'
                    ' has one (set the clock with "fhs time" before nights away)')
            continue
        src = 'flash-guess' if b in guessed else 'flash'
        for seq, r in sorted(by_boot[b]['recs'].items()):
            rows.append(row(t0s[b] + seq * proto.WINDOW_S, b, seq, r, src))
    if guessed:
        log('boots with a guessed time (clock never set): ' + ' '.join(map(str, sorted(guessed))))
    return rows


def split_nights(rows):
    """{evening date: rows}"""
    out = {}
    for r in rows:
        out.setdefault(night_of(float(r['t'])), []).append(r)
    return out


def night_csv(root, day):
    return os.path.join(root, '%s_night' % day.isoformat(), 'work', 'sensor.csv')
