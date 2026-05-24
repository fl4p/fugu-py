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

    def __init__(self, transport: Transport, eol="\r\n", on_line=None, maxlines=2000,
                 wait_banner=False, banner_marker="to disconnect.", banner_timeout=2.0,
                 min_post_connect=1.2):
        self.transport = transport
        self.eol = eol
        self.on_line = on_line
        self.maxlines = maxlines
        self.banner_marker = banner_marker
        self.banner_timeout = banner_timeout
        # Earliest moment a first command can safely be written after open(). ESPTelnet drops
        # any input that arrives in roughly the first second after TCP connect (the banner
        # finishes far earlier — ~100 ms — but the input pump isn't ready yet; bytes sent
        # before then are silently swallowed or coalesced). Measured threshold on flat: writes
        # at 0.92 s dropped, writes at 1.11 s landed. 1.2 s gives margin.
        self.min_post_connect = min_post_connect
        # One-shot guard before the first command. For non-banner transports (serial, MQTT,
        # BLE) leave wait_banner=False; the guard stays disarmed and command() runs as before.
        self._needs_banner = bool(wait_banner)
        self._buf = b''
        self._lines: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        transport.open()
        self._connect_time = time.monotonic()
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

    def reconnect(self):
        """Tear down and re-open the transport, restarting the reader thread if it died."""
        try:
            self.transport.close()
        except Exception as e:
            logger.debug("reconnect close: %s", e)
        self.transport.open()
        self._connect_time = time.monotonic()
        # re-arm the banner wait so the next command rides the new connection's handshake
        self._needs_banner = bool(self.banner_marker)
        if not self._reader.is_alive():
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()

    def wait_for_banner(self, marker: str = None, timeout: float = None) -> bool:
        """Block until a line containing `marker` arrives AND `min_post_connect` has elapsed
        since open(); returns True if the marker was seen, False on timeout.

        Used as a one-shot handshake settle before the first command on transports where the
        device sends a banner (telnet's "Welcome … (Use ^] + q  to disconnect.)"). on_line still
        fires for the consumed lines from the read thread, so interactive callers don't lose them.
        The post-connect floor matters because the banner arrives well before ESPTelnet starts
        accepting input (~100 ms vs ~1 s on flat); without it, commands written right after the
        marker still get dropped.
        """
        marker = marker if marker is not None else self.banner_marker
        timeout = timeout if timeout is not None else self.banner_timeout
        deadline = time.monotonic() + timeout
        seen = False
        while True:
            if seen and time.monotonic() - self._connect_time >= self.min_post_connect:
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                # On timeout, still honor the post-connect floor so the caller's first write
                # doesn't race the device even if the banner never arrives.
                floor = self._connect_time + self.min_post_connect - time.monotonic()
                if floor > 0:
                    time.sleep(floor)
                return seen
            try:
                line = self._lines.get(timeout=remaining)
            except queue.Empty:
                continue
            if marker in line:
                seen = True

    def recover(self, budget=120.0, probe_timeout=5.0) -> bool:
        """Reconnect a dropped link and wait for the device to answer, up to `budget` s.

        Reconnects with exponential backoff (1 s, capped at 15 s) and probes via `wait_ready` each
        round, riding out a router/NAT outage that outlasts `command`'s built-in retries. Returns
        True once the console answers, False if the budget runs out.
        """
        deadline = time.monotonic() + budget
        delay = 1.0
        while time.monotonic() < deadline:
            try:
                self.reconnect()
                if self.wait_ready(timeout=probe_timeout):
                    return True
            except Exception as e:
                logger.warning("recover attempt failed: %s", e)
            time.sleep(delay)
            delay = min(delay * 2, 15.0)
        return False

    def command(self, cmd: str, timeout: float = 4.0, retry=False, recover=0.0) -> Reply:
        """Send `cmd`, collect reply lines until the OK/ERR marker, a rejection, or timeout.

        `retry` reconnects and retries on a transport error or timeout, backing off between
        attempts: pass an int for the attempt count, or True for a default of 5. `recover` is a
        seconds budget for a deeper last resort: if the fast retries are exhausted, ride out a
        longer outage via `recover()` and re-issue the command once. Still raises / returns the
        final timed-out Reply if the device never comes back.
        """
        if not retry and not recover:
            return self._command(cmd, timeout)
        attempts = (retry if isinstance(retry, int) and not isinstance(retry, bool)
                    else (5 if retry else 1))
        backoff, last = 0.5, None
        for attempt in range(1, attempts + 1):
            try:
                reply = self._command(cmd, timeout)
                if not reply.timed_out:
                    return reply
                last = reply
                logger.warning("command %r timed out (%d/%d)", cmd, attempt, attempts)
            except Exception as e:
                last = e
                logger.warning("command %r failed: %s (%d/%d)", cmd, e, attempt, attempts)
            if attempt < attempts:
                try:
                    self.reconnect()
                except Exception as e:
                    logger.warning("reconnect failed: %s", e)
                time.sleep(backoff)
                backoff = min(backoff * 2, 8.0)
        if recover and self.recover(recover):
            try:
                reply = self._command(cmd, timeout)
                if not reply.timed_out:
                    return reply
                last = reply
            except Exception as e:
                last = e
        if isinstance(last, Exception):
            raise last
        return last  # the final timed-out Reply

    def _command(self, cmd: str, timeout: float) -> Reply:
        if self._needs_banner:
            self._needs_banner = False  # one-shot, regardless of outcome
            self.wait_for_banner()
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
