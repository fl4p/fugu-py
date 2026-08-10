import logging
import math
import time


def get_logger(verbose=False):
    # log_format = '%(asctime)s %(levelname)-6s [%(filename)s:%(lineno)d] %(message)s'
    log_format = '%(asctime)s %(levelname)s [%(module)s] %(message)s'
    if verbose:
        level = logging.DEBUG
    else:
        level = logging.INFO

    logging.basicConfig(level=level, format=log_format, datefmt='%H:%M:%S')
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)

    return logger


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
    t_start = time.time()
    remaining = num
    last_press = None
    t0 = t_start
    with tqdm.tqdm(total=seconds, desc='measurement ..', position=1, leave=False) as pbar:
        while seconds > 0:
            try:
                t0 = time.time()
                for i in range(int(seconds)):
                    time.sleep(1)
                    if poll and not poll():
                        print("poll did not return true")
                        return False
                    passed = time.time() - t_start
                    pbar.update(int(passed) - pbar.n)
                break
            except KeyboardInterrupt:
                now = time.time()
                if last_press is None or (now - last_press) >= burst_s:
                    remaining = num          # a new burst, this press is its first
                remaining -= 1
                last_press = now
                if remaining <= 0:
                    raise
                seconds -= now - t0
                print('\ninterrupt %d x again within %gs to cancel. otherwise going to sleep %d'
                      's more ..' % (remaining, burst_s, round(seconds)))
    return True
