import queue
import re
import threading
import time

from .transport import Transport
from .util import get_logger

logger = get_logger()

_ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
# the firmware's final-else when it doesn't recognise a command
REJECT = "unknown or unexpected command"


class Reply(list):
    """Reply lines of a command, plus how the command terminated.

    Behaves as a plain list of lines (the OK/ERR completion marker is not included); `ok`,
    `rejected` and `timed_out` say which terminated the collection.
    """

    def __init__(self, lines=(), ok=False, rejected=False, timed_out=False):
        super().__init__(lines)
        self.ok = ok
        self.rejected = rejected
        self.timed_out = timed_out

    @property
    def text(self) -> str:
        return "\n".join(self)


class Console:
    """Line-oriented console over any Transport.

    A background thread drains the transport, strips ANSI, and assembles whole lines into a queue
    (the firmware streams periodic status lines between commands). `command()` sends one command
    and gathers reply lines until the firmware's 'OK: <cmd>' / 'ERR: <cmd>' completion marker, a
    rejection, or a timeout. Works the same over serial, TCP/telnet, or BLE.
    """

    def __init__(self, transport: Transport, eol="\r\n", on_line=None, maxlines=2000):
        self.transport = transport
        self.eol = eol
        self.on_line = on_line
        self.maxlines = maxlines
        self._buf = b''
        self._lines: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        transport.open()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def is_alive(self) -> bool:
        return self._reader.is_alive()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                chunk = self.transport.read()
            except Exception as e:
                logger.debug("console read error: %s", e)
                break
            if not chunk:
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                raw, self._buf = self._buf.split(b"\n", 1)
                line = _ANSI.sub("", raw.decode("utf-8", "replace")).rstrip("\r")
                self._lines.put(line)
                # cap memory when nobody is consuming (a monitoring-only owner that taps via
                # on_line but never calls command()); keep the most recent lines
                while self.maxlines and self._lines.qsize() > self.maxlines:
                    try:
                        self._lines.get_nowait()
                    except queue.Empty:
                        break
                if self.on_line:
                    self.on_line(line)

    def drain(self):
        """Discard buffered lines (e.g. status lines streamed before the next command)."""
        while True:
            try:
                self._lines.get_nowait()
            except queue.Empty:
                return

    def write(self, text: str):
        self.transport.write(text.encode("utf-8"))

    def command(self, cmd: str, timeout: float = 4.0) -> Reply:
        self.drain()
        self.write(cmd + self.eol)
        ok_marker, err_marker = "OK: " + cmd, "ERR: " + cmd
        lines = []
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return Reply(lines, timed_out=True)
            try:
                line = self._lines.get(timeout=remaining)
            except queue.Empty:
                return Reply(lines, timed_out=True)
            if ok_marker in line:
                return Reply(lines, ok=True)
            if err_marker in line or REJECT in line:
                return Reply(lines, rejected=True)
            lines.append(line)

    def wait_ready(self, probe="mem", timeout=30.0, probe_timeout=2.0) -> bool:
        """Poll `probe` until it is acknowledged — covers the boot / ADC-calibration window."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.command(probe, timeout=probe_timeout).ok:
                return True
        return False

    def close(self):
        self._stop.set()
        try:
            self.transport.close()  # unblocks the reader's transport.read()
        except Exception:
            pass
        if threading.current_thread() is not self._reader:
            self._reader.join(timeout=2)
