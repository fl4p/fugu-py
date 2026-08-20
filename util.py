import logging
import logging.handlers
import math
import os
import time

LOG_FORMAT = '%(asctime)s %(levelname)s [%(module)s] %(message)s'
# Full date on disk. The console format carries only %H:%M:%S, which is fine while someone is
# watching it, and useless in a file that outlives the day it was written.
FILE_LOG_FORMAT = '%(asctime)s %(levelname)s [%(module)s] %(message)s'
FILE_DATEFMT = '%Y-%m-%d %H:%M:%S'


def get_logger(verbose=False, log_path=None, file_level=logging.DEBUG,
               max_bytes=8 << 20, backups=5):
    """Configure and return the root logger. Optionally ALSO write it to `log_path`.

    Console behaviour is unchanged when `log_path` is None, which is every existing caller.

    `log_path` exists because the console traffic from a Fugu is the highest-value diagnostic
    this library produces -- the raw `Ser:` firmware lines: duty clamping ("dc: out of range
    [0,3751]"), sync-rect transitions, converter enable/disable, the trip/backoff fault loop --
    and it went to stderr and nowhere else. That is tolerable for a script somebody is watching
    and wrong for a LONG-LIVED SESSION: a process that holds two converters across many runs
    and parks them on an idle watchdog is the one whose history matters most, and it was the
    only one keeping none. Correlating a refused measurement against what the firmware actually
    did at that instant is not possible after the fact if the lines were never written down.

    Raises OSError if `log_path` cannot be opened, after trying to create its parent directory.
    Deliberately NOT swallowed: a caller that asked for a log and did not get one must not
    proceed believing it has one. Whether that is fatal is the CALLER's decision -- a library
    has no business deciding that a missing log file should stop someone else's bench -- so
    wrap the call if you want to continue without it, and say so when you do.
    """
    logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO,
                        format=LOG_FORMAT, datefmt='%H:%M:%S')
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)
    if log_path:
        add_file_log(log_path, level=file_level, max_bytes=max_bytes, backups=backups)
    return logger


def add_file_log(path, level=logging.DEBUG, max_bytes=8 << 20, backups=5, logger=None):
    """Also write the log to `path`, rotating. -> the resolved absolute path.

    ROTATES because the caller is expected to be long-running: a session streaming ~1 Hz status
    lines plus console output from two devices grows without bound otherwise, and an unattended
    bench filling its own disk is a worse failure than a truncated log.

    The file handler defaults to DEBUG even when the console is at INFO -- full detail on disk,
    readable summary on screen, which is the point of having a file at all. Records are filtered
    by the LOGGER's level before any handler sees them, so the root level has to be lowered for
    that to work; every existing StreamHandler is first pinned to its current effective level so
    CONSOLE OUTPUT DOES NOT CHANGE. Without that pinning, adding a log file would silently start
    dumping debug traffic onto the terminal of every caller.

    Idempotent per resolved path: calling it twice does not double-write. `get_logger` is invoked
    at import time by half the modules in this package, so a second call is the normal case, not
    an abuse.
    """
    logger = logger or logging.getLogger()
    resolved = os.path.abspath(os.path.expanduser(path))
    for h in logger.handlers:
        if isinstance(h, logging.FileHandler) and \
                os.path.abspath(getattr(h, "baseFilename", "")) == resolved:
            return resolved
    parent = os.path.dirname(resolved)
    if parent:
        os.makedirs(parent, exist_ok=True)
    handler = logging.handlers.RotatingFileHandler(
        resolved, maxBytes=max_bytes, backupCount=backups)
    handler.setLevel(level)
    handler.setFormatter(logging.Formatter(FILE_LOG_FORMAT, datefmt=FILE_DATEFMT))
    if level < logger.level:
        # Pin the console FIRST, then lower the gate. Order matters: doing it the other way
        # round leaves a window in which the stream handler inherits DEBUG.
        for h in logger.handlers:
            # FileHandler SUBCLASSES StreamHandler, so exclude it explicitly -- otherwise an
            # earlier file log would get pinned to the console level and stop recording debug.
            if isinstance(h, logging.StreamHandler) \
                    and not isinstance(h, logging.FileHandler) \
                    and h.level == logging.NOTSET:
                h.setLevel(logger.level)
        logger.setLevel(level)
    logger.addHandler(handler)
    return resolved


def round_to_n(x, n):
    if not x or isinstance(x, str) or not math.isfinite(x):
        return x

    try:
        f = round(x, -int(math.floor(math.log10(abs(x)))) + (n - 1))
        if isinstance(f, float) and f.is_integer():
            return int(f)
        return f
    except ValueError as e:
        raise e


def num2str(x, n=None):
    if n is not None:
        x = round_to_n(x, n)
    s = str(x)
    if s.endswith('.0'):
        s = s[:-2]
    return s


def round_to_n_dec(x, n):
    x = round_to_n(x, n)
    if not x or isinstance(x, str) or not math.isfinite(x):
        return str(x)

    ax = abs(x)

    if ax < 1e-20:
        return '0' if x >= 0 else '-0'
    elif ax < 999e-12:
        return num2str(x * 1e12, n) + 'p'
    elif ax < 999e-9:
        return num2str(x * 1e9, n) + 'n'
    elif ax < 999e-6:
        return num2str(x * 1e6, n) + 'µ'
    elif ax < 999e-3:
        return num2str(x * 1e3, n) + 'm'
    elif ax > 0.999e6:
        return num2str(x * 1e-6, n) + 'M'
    elif ax > 0.999e3:
        return num2str(x * 1e-3, n) + 'k'
    # elif x > 9
    else:
        return num2str(x, n)


class TerminationRequested(BaseException):
    pass


def sleep_confirm_interrupt(seconds, num=3, poll=None, burst_s=5) -> bool:
    """Sleep, polling once a second, and cancel only on `num` interrupts in quick succession.

    Deliberately hard to interrupt: this is what runs while a converter is at power, and a
    stray Ctrl+C must not drop the run on a path that skips the ramp-down.

    `num` means what it says -- 3 presses within `burst_s` of EACH OTHER. It previously took
    four, because the elapsed time was measured from the start of the dwell rather than from
    the previous press: the first interrupt into a 60 s dwell had st >= 5, so `st < 5 and
    (num := num - 1)` short-circuited before the walrus, the counter never decremented, and
    that press was silently discarded. The printed countdown was off by one for the same
    reason. Measured 2026-08-08: bursts of three never cancelled, five did.

    Waiting longer than `burst_s` now genuinely resets the count (the old TODO), so an
    accidental press minutes earlier cannot combine with two deliberate ones later.
    """
    import tqdm
    total = left = float(seconds)
    remaining = num
    last_press = None
    t0 = time.time()
    with tqdm.tqdm(total=total, desc='measurement ..', position=1, leave=False) as pbar:
        while left > 0:
            try:
                t0 = time.time()
                sleep_s = min(1.0, left)
                time.sleep(sleep_s)
                if poll and not poll():
                    print("poll did not return true")
                    return False
                elapsed = max(0.0, time.time() - t0)
                left -= max(sleep_s, elapsed)
                pbar.update(min(total, total - max(left, 0.0)) - pbar.n)
            except KeyboardInterrupt:
                now = time.time()
                if last_press is None or (now - last_press) >= burst_s:
                    remaining = num          # a new burst, this press is its first
                remaining -= 1
                last_press = now
                if remaining <= 0:
                    raise
                left -= max(0.0, now - t0)
                print('\ninterrupt %d x again within %gs to cancel. otherwise going to sleep %d'
                      's more ..' % (remaining, burst_s, round(max(left, 0.0))))
    return True
