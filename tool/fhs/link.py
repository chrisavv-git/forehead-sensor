"""Text links to the sensor: USB serial and Bluetooth (Nordic UART service). Both give whole lines."""
import asyncio
import glob
import threading

NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'   # laptop -> device (write)
NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'   # device -> laptop (notify)
ESPRESSIF_USB = (0x303A, 0x1001)      # the T-Watch S3: the ESP32-S3's own USB serial
NAME_PREFIX = 'Forehead-'


class LinkLost(Exception):
    pass


class Link:
    """lines in, lines out; read_line() raises LinkLost once the link is gone"""
    trusted = False   # USB needs no key

    def __init__(self):
        self.q = asyncio.Queue()
        self._part = b''
        self.lost = False
        self.loop = None

    def _feed(self, data: bytes):   # may be called from another thread via call_soon_threadsafe
        self._part += data
        *lines, self._part = self._part.split(b'\n')
        for ln in lines:
            self.q.put_nowait(ln.rstrip(b'\r').decode('ascii', 'replace'))

    def _gone(self):
        self.lost = True
        self.q.put_nowait(None)

    async def read_line(self, timeout=None):
        """next line, or None after timeout seconds"""
        if self.lost and self.q.empty():
            raise LinkLost()
        try:
            ln = await asyncio.wait_for(self.q.get(), timeout) if timeout is not None else await self.q.get()
        except asyncio.TimeoutError:
            return None
        if ln is None:
            raise LinkLost()
        return ln


class SerialLink(Link):
    def __init__(self, port, trusted=True):
        super().__init__()
        self.port, self.trusted, self.ser = port, trusted, None

    async def open(self):
        import serial
        self.loop = asyncio.get_running_loop()
        self.ser = serial.Serial(self.port, 115200, timeout=0.2)   # opening raises DTR: the chip's "connected"
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self._stop:
            try:
                data = self.ser.read(4096)
            except Exception:
                self.loop.call_soon_threadsafe(self._gone)
                return
            if data:
                self.loop.call_soon_threadsafe(self._feed, data)

    async def write_line(self, s: str):
        if self.lost:
            raise LinkLost()
        try:
            await asyncio.get_running_loop().run_in_executor(None, self.ser.write, (s + '\n').encode())
        except Exception as e:
            self._gone()
            raise LinkLost() from e

    async def close(self):
        self._stop = True
        if self.ser:
            await asyncio.sleep(0.25)
            self.ser.close()


class BleLink(Link):
    def __init__(self, address=None, name_prefix=NAME_PREFIX, scan_timeout=10.0):
        super().__init__()
        self.address, self.name_prefix, self.scan_timeout, self.client = address, name_prefix, scan_timeout, None
        self.name = None

    async def open(self):
        from bleak import BleakClient, BleakScanner
        self.loop = asyncio.get_running_loop()
        if self.address:
            dev = await BleakScanner.find_device_by_address(self.address, timeout=self.scan_timeout)
        else:
            dev = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name or ad.local_name or '').startswith(self.name_prefix), timeout=self.scan_timeout)
        if dev is None:
            raise LinkLost('sensor not found over Bluetooth' + (f' ({self.address})' if self.address else ''))
        self.address, self.name = dev.address, dev.name
        self.client = BleakClient(dev, disconnected_callback=lambda c: self.loop.call_soon_threadsafe(self._gone))
        await self.client.connect()
        await self.client.start_notify(NUS_TX, lambda _c, data: self._feed(bytes(data)))

    async def write_line(self, s: str):
        if self.lost or not self.client or not self.client.is_connected:
            raise LinkLost()
        data = (s + '\n').encode()
        size = max(20, min(180, (self.client.mtu_size or 23) - 3))
        try:
            for i in range(0, len(data), size):
                await self.client.write_gatt_char(NUS_RX, data[i:i + size], response=False)
        except Exception as e:
            self._gone()
            raise LinkLost() from e

    async def close(self):
        if self.client:
            try:
                await self.client.disconnect()
            except Exception:
                pass


def find_usb():
    """the sensor's USB serial port, or None"""
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if (p.vid, p.pid) == ESPRESSIF_USB:
                return p.device
    except Exception:
        pass
    by_id = sorted(glob.glob('/dev/serial/by-id/*Espressif*'))
    return by_id[0] if by_id else None


async def scan(timeout=8.0):
    """[(address, name, rssi)] of sensors in range"""
    from bleak import BleakScanner
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    return sorted(((d.address, d.name or ad.local_name, ad.rssi) for d, ad in found.values()
                   if (d.name or ad.local_name or '').startswith(NAME_PREFIX)), key=lambda x: -x[2])
