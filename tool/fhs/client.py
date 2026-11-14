"""Talks to the sensor over a Link: commands with their replies, plus the streams (records, raw samples).
Lines come in the order the device sent them, so a command's reply lines (S, P, I, B ...) all end up in
self.replies before its 'OK <command>'; records go to self.records whenever they come."""
import asyncio
import time

from . import proto
from .link import LinkLost


class DeviceError(Exception):
    pass


class Client:
    def __init__(self, link, key=None, log=print):
        self.link, self.key, self.log = link, key, log
        self.replies = asyncio.Queue()
        self.records = asyncio.Queue()     # (boot, seq, Record); None once the link is lost
        self.events = asyncio.Queue()      # C (caught up), X/M (raw samples)
        self._task = None

    async def __aenter__(self):
        await self.link.open()
        self._task = asyncio.create_task(self._pump())
        if not self.link.trusted:
            if not self.key:
                raise DeviceError('this link needs the key: run "fhs pair" with the sensor on USB first')
            await self.cmd('auth ' + self.key)
        return self

    async def __aexit__(self, *exc):
        if self._task:
            self._task.cancel()
        await self.link.close()

    @property
    def lost(self):
        return self.link.lost

    async def _pump(self):
        try:
            while True:
                ln = await self.link.read_line()
                if not ln:
                    continue
                k = ln[0]
                if k == 'R':
                    r = proto.parse_R(ln)
                    if r:
                        self.records.put_nowait(r)
                elif k == 'N':
                    self.log('sensor: ' + ln[2:])
                elif ln[:2] in ('C ', 'X ', 'M '):
                    self.events.put_nowait(ln)
                else:
                    self.replies.put_nowait(ln)
        except LinkLost:
            for q in (self.replies, self.records, self.events):
                q.put_nowait(None)

    async def cmd(self, line, timeout=5.0):
        """send a command; returns its reply lines up to and including 'OK <command>'. timeout = the longest wait
        for the next line. Raises DeviceError on 'ERR ...' or silence, LinkLost if the link goes."""
        while not self.replies.empty():   # left over from a command that timed out
            if self.replies.get_nowait() is None:
                raise LinkLost()
        await self.link.write_line(line)
        word, out = line.split()[0], []
        while True:
            try:
                ln = await asyncio.wait_for(self.replies.get(), timeout)
            except asyncio.TimeoutError:
                raise DeviceError(f'no reply to "{word}"') from None
            if ln is None:
                raise LinkLost()
            out.append(ln)
            if ln.startswith('ERR'):
                raise DeviceError(ln)
            if ln == 'OK ' + word or ln.startswith('OK ' + word + ' '):
                return out

    async def get_status(self):
        st = {}
        for ln in await self.cmd('status'):
            if ln.startswith('S hw '):
                st['hw'] = ln[5:]
            elif ln.startswith('S '):
                st.update(proto.parse_kv(ln))
        return st

    async def get_params(self):
        for ln in await self.cmd('get'):
            if ln.startswith('P '):
                return proto.parse_kv(ln)
        return {}

    async def set_time(self):
        """set the device clock from the laptop's; returns (boot, unix s of record 0 of this boot)"""
        now = time.time()
        tz_min = int(round(time.localtime(now).tm_gmtoff / 60))
        r = proto.parse_kv((await self.cmd(f'time {int(now * 1000)} {tz_min}'))[-1])
        return int(r['boot']), int(r['t0']) / 1000

    async def blocks(self):
        """[dict(sector, block_no, boot, first_seq, n, t0_ms, open)], oldest first"""
        out = []
        for ln in await self.cmd('blocks', timeout=10):
            p = ln.split()
            if p[0] == 'I' and len(p) == 8:
                out.append(dict(sector=int(p[1]), block_no=int(p[2]), boot=int(p[3]), first_seq=int(p[4]),
                                n=int(p[5]), t0_ms=int(p[6]), open=p[7] == '1'))
        return out

    async def dump(self, from_block=0, count=None):
        """{block_no: 4096 bytes} of every block from from_block on (count of them, or all); incomplete ones are
        left out"""
        parts = {}
        for ln in await self.cmd(f'dump {from_block}' + (f' {count}' if count else ''), timeout=20):
            p = ln.split()
            if p[0] == 'B' and len(p) == 4:
                parts.setdefault(int(p[1]), {})[int(p[2])] = bytes.fromhex(p[3])
        blocks = {}
        for bn, ch in parts.items():
            if sorted(ch) == list(range(0, proto.BLOCK_SIZE, 256)) and all(len(v) == 256 for v in ch.values()):
                blocks[bn] = b''.join(ch[o] for o in range(0, proto.BLOCK_SIZE, 256))
            else:
                self.log(f'block {bn}: incomplete, skipped')
        return blocks

    async def next_event(self, kinds, timeout):
        """next C/X/M line whose first letter is in kinds, or None after timeout"""
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return None
            try:
                ln = await asyncio.wait_for(self.events.get(), left)
            except asyncio.TimeoutError:
                return None
            if ln is None:
                raise LinkLost()
            if ln[0] in kinds:
                return ln
