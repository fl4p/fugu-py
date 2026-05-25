import asyncio
import glob
import queue
import socket
import sys
import threading
import time
from typing import Optional

import serial  # pyserial

from .util import get_logger

logger = get_logger()


class Transport(object):

    def open(self):
        raise NotImplementedError()

    def read(self) -> bytes:
        raise NotImplementedError()

    def write(self, data: bytes):
        raise NotImplementedError()

    def close(self):
        raise NotImplementedError()


class SerialTransport(Transport):

    def __init__(self, port, baud=115200, timeout: Optional[float] = None):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self):
        if self.ser and self.ser.is_open:
            return
        port = self.port
        if '*' in port:
            port = glob.glob(port)[0]
        logger.info(f'opening serial port {port} @ {self.baud}')
        self.ser = serial.Serial(port, baudrate=self.baud, timeout=self.timeout)

    def write(self, data: bytes):
        self.ser.write(data)

    def read(self) -> Optional[bytes]:
        if self.ser.is_open and self.ser.readable():
            return self.ser.readline()
        return None


class SocketTransport(Transport):
    DEFAULT_PORT = 23  # telnet

    def __init__(self, ip, port=DEFAULT_PORT, timeout=4, is_telnet=True):
        self.addr = (ip, port)
        self.timeout = timeout
        self.sock = None
        self.is_telnet = is_telnet
        self.t_last_comm = time.time()

    def open(self):
        # fresh socket every time so open() can be used to reconnect after a drop
        self.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        # TCP keepalive so we notice silently-dead peers (e.g. ESP32 reboot mid-OTA without
        # a clean FIN) within ~10 s instead of waiting on the default ~2 h.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        for _name, _val in (('TCP_KEEPIDLE', 3), ('TCP_KEEPALIVE', 3),
                            ('TCP_KEEPINTVL', 2), ('TCP_KEEPCNT', 3)):
            _opt = getattr(socket, _name, None)
            if _opt is not None:
                try:
                    self.sock.setsockopt(socket.IPPROTO_TCP, _opt, _val)
                except OSError:
                    pass
        logger.info('connecting to %s:%u', *self.addr)
        self.sock.connect(self.addr)
        self.t_last_comm = time.time()
        logger.info('connected to %s:%u', *self.addr)

    def close(self):
        self.t_last_comm = 0
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)  # wake a recv() blocked in the reader thread
            except OSError:
                pass
            self.sock.close()
            self.sock = None

    def read(self):
        try:
            r = self.sock.recv(1024)
            if not r:
                # recv()==b'' on a connected stream socket is EOF (peer sent FIN).
                # Tear down so callers stop polling and the reader thread exits.
                self.close()
                return b''
            self.t_last_comm = time.time()
            if time.time() - self.t_last_comm > 1:
                # check conn health
                if self.is_telnet:
                    self.write(bytes([255, 241]))  # send telnet NOP to probe conn TODO Are you there 246 ?
            return r
        except socket.timeout:
            # recv() hit the socket-level timeout, not a disconnect — return empty so the read
            # loop just keeps polling. Long-running commands (notably `ota <url>` during the
            # HTTP connect window before any progress logs flow) can go silent for > self.timeout
            # seconds; without this, the reader thread dies and Console.command() waits forever
            # for an OK marker that's still queued on the device.
            return b''
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            # keepalive failure surfaces as ETIMEDOUT here; treat any of these as disconnect
            print(self.sock, type(e).__name__, e)
            self.close()
            raise

    def write(self, data):
        i = self.sock.send(data)
        if i > 0:
            self.t_last_comm = time.time()
        return i

    def check_connection(self) -> bool:
        if self.sock is None:
            return False
        if time.time() - self.t_last_comm < 4:
            return True

        try:
            # peek without blocking and without consuming the buffer
            data = self.sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                # peer closed cleanly (FIN). recv() returning 0 on a stream socket = EOF.
                self.close()
                return False
        except ConnectionResetError as e:
            print(type(e), e)
            self.close()
            return False  # socket was closed for some other reason
        except BlockingIOError:
            return True  # socket is open and reading from it would block
        except OSError as e:
            print(type(e), e)
            self.close()
            return False  # 'Bad file descriptor' or keepalive ETIMEDOUT
        except Exception as e:
            print("unexpected exception when checking if a socket is closed", type(e), e)
            return True
        return True


class BleTransport(Transport):
    """Nordic UART Service (NUS) link to the firmware's BleConsoleService.

    Bridges bleak's asyncio API to the synchronous Transport interface: a private event loop runs
    in a background thread, notifications are pushed into a byte queue, and read() hands back one
    decoded line at a time (ANSI kept raw, like the serial/socket transports). Requires `bleak`.
    """

    NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
    RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (host -> device)
    TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (device -> host)
    ATT_CHUNK = 20  # conservative pre-MTU-negotiation ATT payload

    def __init__(self, name="fugu", address=None, scan_timeout=10.0, connect_retries=3,
                 read_timeout=0.3):
        self.name = name
        self.address = address
        self.scan_timeout = scan_timeout
        self.connect_retries = connect_retries
        self.read_timeout = read_timeout
        self._rx: "queue.Queue[bytes]" = queue.Queue()
        self._buf = b''
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._client = None

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result()

    def open(self):
        if self._client is not None:
            return
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()
        self._submit(self._connect())

    async def _find_device(self):
        from bleak import BleakScanner
        if self.address:
            return await BleakScanner.find_device_by_address(self.address, timeout=self.scan_timeout)
        name = (self.name or "").lower()

        def match(d, adv):
            has_nus = self.NUS_SERVICE in (s.lower() for s in (adv.service_uuids or []))
            # The firmware carries the NUS UUID in the scan response, which macOS surfaces only
            # sporadically; when a name filter is given, trust it (NUS is verified at connect).
            if name:
                return name in (d.name or "").lower()
            return has_nus

        return await BleakScanner.find_device_by_filter(match, timeout=self.scan_timeout)

    async def _connect(self):
        from bleak import BleakClient
        dev = await self._find_device()
        if dev is None:
            raise RuntimeError("no device advertising NUS found (is it advertising? `svc on ble`)")
        logger.info("connecting to %s [%s]", dev.name, dev.address)
        # macOS can keep a stale bond after a reflash ("Peer removed pairing information"); the
        # failed attempt usually drops the bond, so retry a couple of times.
        last = None
        client = BleakClient(dev)
        for attempt in range(1, self.connect_retries + 1):
            try:
                await client.connect()
                break
            except Exception as e:
                last = e
                logger.warning("connect attempt %d failed: %s", attempt, e)
                if attempt == self.connect_retries:
                    raise RuntimeError(f"could not connect after {attempt} attempts: {last}")
                await asyncio.sleep(1.5)
        try:
            await client.start_notify(self.TX_UUID, self._on_notify)
        except Exception as e:
            self._explain_cccd_failure(dev, e)
            raise
        self._client = client

    @staticmethod
    def _explain_cccd_failure(dev, e):
        # macOS caches the GATT table per bonded peer; after a BLE OTA shifts attribute handles the
        # cached CCCD handle no longer maps to the TX descriptor and the subscribe write is rejected
        # with ATT code 3 ("Writing is not permitted"). Forgetting the bond clears the stale cache.
        msg = str(e).lower()
        if sys.platform != "darwin" or ("not permitted" not in msg and "code=3" not in msg):
            return
        logger.error(
            "enabling notifications failed — this is macOS using a stale cached GATT for a bonded\n"
            "  device (the cache no longer matches the firmware after a BLE OTA), not a device fault.\n"
            "  Clear it and reconnect:\n"
            "    blueutil --unpair $(blueutil --paired | sed -n 's/.*address: \\([^,]*\\),.*name: \"%s\".*/\\1/p')\n"
            "    # or: System Settings > Bluetooth > %s > Forget This Device\n"
            "  then toggle Bluetooth off/on and reconnect.",
            dev.name or "<device>", dev.name or "the device")

    def _on_notify(self, _char, data: bytearray):
        self._rx.put(bytes(data))

    def read(self) -> bytes:
        deadline = time.time() + self.read_timeout
        while True:
            nl = self._buf.find(b'\n')
            if nl >= 0:
                line, self._buf = self._buf[:nl + 1], self._buf[nl + 1:]
                return line
            remaining = deadline - time.time()
            if remaining <= 0:
                return b''
            try:
                self._buf += self._rx.get(timeout=remaining)
            except queue.Empty:
                return b''

    def write(self, data: bytes):
        # write-with-response: the RX char requires an encrypted write under
        # ble_security=justworks|passkey, and some stacks drop write-without-response there.
        for off in range(0, len(data), self.ATT_CHUNK):
            self._submit(self._client.write_gatt_char(
                self.RX_UUID, data[off:off + self.ATT_CHUNK], response=True))

    def close(self):
        if self._client is not None:
            try:
                self._submit(self._client.disconnect())
            except Exception as e:
                logger.debug("ble disconnect: %s", e)
            self._client = None
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=2)


class MqttTransport(Transport):
    """Console over MQTT, via the broker the device is configured to use.

    The firmware mirrors all console output to `pv/log/<hostname>` and (with the command
    subscription in `MqttService::onStart`) accepts commands on `pv/log/<hostname>/cmd`; the
    OK/ERR marker comes back on the log topic. `device` is matched as a substring of the
    advertised hostname (like the BLE name filter); the full hostname is learned from the first
    log message and used for the command topic. Requires `paho-mqtt`.

    `writable=False` makes it a read-only monitor — `write()` is a no-op, so it streams the
    device's output without ever publishing commands.
    """

    LOG_ROOT = "pv/log/"

    def __init__(self, host, port=1883, username=None, password=None, device="fugu",
                 writable=True, discover_timeout=10.0, read_timeout=0.3):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.device = device or ""
        self.writable = writable
        self.discover_timeout = discover_timeout
        self.read_timeout = read_timeout
        self.hostname = None  # full device hostname, learned from the first matching log topic
        self._rx: "queue.Queue[bytes]" = queue.Queue()
        self._buf = b''
        self._client = None
        self._learned = threading.Event()

    def open(self):
        if self._client is not None:
            return
        import paho.mqtt.client as mqtt
        from paho.mqtt.enums import CallbackAPIVersion
        c = mqtt.Client(CallbackAPIVersion.VERSION2)
        if self.username:
            c.username_pw_set(self.username, self.password or "")
        c.on_message = self._on_message
        logger.info("connecting to broker %s:%u", self.host, self.port)
        c.connect(self.host, self.port, 60)
        c.subscribe(self.LOG_ROOT + "#")  # output is pv/log/<hostname>, cmd echo is .../cmd
        c.loop_start()
        self._client = c
        # learn the device hostname from its log stream (needed to address the command topic)
        if not self._learned.wait(self.discover_timeout):
            if self.device:
                logger.warning("no log seen on %s*%s*; assuming hostname=%r",
                               self.LOG_ROOT, self.device, self.device)
                self.hostname = self.device
            else:
                raise RuntimeError(f"no device publishing under {self.LOG_ROOT}")

    def _on_message(self, _client, _userdata, msg):
        parts = msg.topic.split("/")
        if len(parts) != 3:
            return  # pv/log/<hostname> only; ignore the deeper .../cmd echo
        hostname = parts[2]
        if self.device.lower() not in hostname.lower():
            return
        if self.hostname is None:
            self.hostname = hostname
            logger.info("device hostname: %s", hostname)
            self._learned.set()
        self._rx.put(msg.payload)

    def read(self) -> bytes:
        deadline = time.time() + self.read_timeout
        while True:
            nl = self._buf.find(b'\n')
            if nl >= 0:
                line, self._buf = self._buf[:nl + 1], self._buf[nl + 1:]
                return line
            remaining = deadline - time.time()
            if remaining <= 0:
                return b''
            try:
                self._buf += self._rx.get(timeout=remaining)
            except queue.Empty:
                return b''

    def write(self, data: bytes):
        if not self.writable:
            return  # read-only monitor
        if not self.hostname:
            raise RuntimeError("device hostname unknown; cannot publish command")
        # MQTT is message-framed, not a byte stream: publish one trimmed command per write
        cmd = data.decode("utf-8", "replace").strip()
        if cmd:
            self._client.publish(f"{self.LOG_ROOT}{self.hostname}/cmd", cmd, qos=0)

    def close(self):
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception as e:
                logger.debug("mqtt disconnect: %s", e)
            self._client = None
