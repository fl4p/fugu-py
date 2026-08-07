"""






"""
import collections
import re
import sys
import time
from math import nan
from typing import Optional, Literal, Union

from .console import Console
from .transport import SocketTransport, Transport, SerialTransport
from .util import get_logger

logger = get_logger()

# 'V=73.6/27.25 I=3.75/ 9.88A 276.3W 53℃54℃ 454sps  0㎅/s CCM(H|L|Lm)= 790|1257|1257 st= MANU,0 lag=3292㎲ N=1192849 rssi=0'
r_float = r'(\d+\.?\d*(e\d+\.?\d*)?|nan)'
RE_PWM = re.compile(
    r'V=\s*(?P<vin>[0-9.]+)\s*/\s*(?P<vout>[0-9.]+).+'
    fr'([0-9.]+)W (?P<tmp_ntc>{r_float})℃(?P<tmp_mcu>{r_float})℃\s.*'
    r'(?P<mode>[CD]CM)\(H\|L\|Lm\)=\s*(?P<ctrl>[0-9]+)\|\s*(?P<sync>[0-9]+)\|\s*(?P<sync_max>[0-9]+)\s.+'
    r'rssi=\s*(?P<rssi>-?[0-9]+)'
)

# Crash/panic markers in the serial log. `rst:0x..` is intentionally NOT here: the bootloader prints
# it on every reset (incl. a clean `restart` and power-on), so it signals a reboot, not a crash.
# Extend as needed.
RE_CRASH = re.compile(
    r"Guru Meditation|panic'?ed|Backtrace:|abort\(\) was called|"
    r"StoreProhibited|LoadProhibited|assert failed"
)


def boost_D2M(d):
    # boost converter duty cycle to ratio
    return 1 / (1 - d)


def boost_M2D(m):
    # boost converter ratio to duty cycle
    return 1 - 1 / m


class PwmState:
    def __init__(self, ccm: Optional[bool], pwm_ctrl, pwm_sync, pwm_sync_max):
        self.ccm = ccm
        self.pwm_ctrl = int(pwm_ctrl)
        self.pwm_sync = int(pwm_sync)
        self.pwm_sync_max = int(pwm_sync_max)

    def __eq__(self, other):
        return (
                self.ccm == other.ccm
                and self.pwm_ctrl == other.pwm_ctrl
                and self.pwm_sync == other.pwm_sync
                and self.pwm_sync_max == other.pwm_sync_max
        )

    def __ne__(self, other):
        return not self.__eq__(other)

    def __repr__(self):
        return f'PwmState(ccm={self.ccm},pwm_ctrl={self.pwm_ctrl},pwm_sync={self.pwm_sync},pwm_sync_max={self.pwm_sync_max})'

    def __str__(self):
        return repr(self)


class FuguDevice:
    # cached state older than this counts as stale (see telemetry_fresh()). The device
    # prints its PWM status line about once a second; keep this well above that period
    # but short enough that a wedged link is caught within one measurement step.
    telemetry_max_age = 10.0

    @staticmethod
    def get_default_serial_port():
        import socket
        return '/dev/ttyACM1' if socket.gethostname() == 'rpi' else '/dev/cu.usbmodem*'

    _warned_set_d = False

    def __init__(self, transport: Transport = None, ip=None, prefix='', block=True,
                 block_timeout=30.0):
        self.pwm_state = PwmState(None, 0, 0, 0)
        # PWM resolution of the active gate driver (ledc: 2047, mcpwm: ~4103), queried
        # lazily from the device (see query_pwm_limits). Raw `dc` counts scale with it.
        self._pwm_max = None
        self._pwm_ctrl_max = None  # the `dc` clamp limit (pwmMax - pwmRectMin)
        self.pwm_freq = None
        self.wifi_rssi = 0
        self.temperatures = []
        self.voltages = []
        # monotonic time of the last parsed telemetry line, see telemetry_fresh()
        self._last_telemetry_t = None

        if ip:
            assert transport is None
            transport = SocketTransport(ip, 23)
        elif transport is None:
            transport = SerialTransport(FuguDevice.get_default_serial_port())

        self.ser_tail = collections.deque(maxlen=20)
        self.prefix = prefix
        self.transport = transport
        self.verbose = False
        self.on_message = None

        # crash/reboot tracking. _last_crash_time is set from the reader thread in _on_line();
        # _last_uptime is updated by has_rebooted()/get_uptime() polls.
        self._last_crash_time = None       # monotonic time a panic marker was last logged
        self._crash_baseline_time = time.monotonic()  # has_crashed() reset point
        self._rebooted = False             # sticky reboot flag, cleared by has_rebooted(reset=True)
        self._last_uptime = None           # last device uptime (s) seen, for regression detection

        self.is_open = True
        # Console owns the reader thread and line assembly. _on_line taps every line for the
        # continuous PWM-status parse / logging; command replies come back via console.command().
        self.console = Console(transport, eol='\n', on_line=self._on_line)

        if block:
            try:
                self.wait_for_pwm_state(timeout=block_timeout)
            except BaseException:
                # the console owns a reader thread and the transport; without this an
                # __init__ that raises (or a Ctrl+C in the wait) leaks both, and the next
                # connect attempt fails on the still-allocated link
                self.console.close()
                raise

    def open(self):
        assert not self.is_open
        raise NotImplementedError()

    def wait_for_pwm_state(self, timeout=30.0):
        """Block until the first status line has been parsed.

        Times out rather than waiting forever: a device that is powered but not printing a
        parseable status line (wrong baud, a firmware that never starts the converter, a BLE
        link that connected but delivers nothing) would otherwise hang here with no exception
        ever reaching the caller's unwind -- so the converter is never brought down.
        """
        assert self.console.is_alive()
        self.pwm_state = PwmState(None, 0, 0, 0)
        t0 = time.monotonic()
        while self.pwm_state.ccm is None:
            if timeout is not None and time.monotonic() - t0 > timeout:
                raise TimeoutError(
                    '%sno parseable status line after %.0fs -- device silent or not printing '
                    'telemetry (last lines: %s)'
                    % (self.prefix, timeout, list(self.ser_tail)[-3:] or 'none'))
            if not self.console.is_alive():
                raise ConnectionError(self.prefix + 'reader thread died while waiting for '
                                                    'the first status line')
            time.sleep(0.1)

    def close(self, close_transport=True, join_rx=True):
        self.is_open = False
        self.pwm_state = PwmState(None, 0, 0, 0)
        self._last_telemetry_t = None  # a closed device has no current state
        self._pwm_max = self._pwm_ctrl_max = None
        self.console.close()

    def _on_line(self, rx: str):
        """Tap for every assembled line (already ANSI-stripped): keep PWM state / temps / voltages
        current and surface warnings, the way the old receive loop did."""
        if not rx:
            return

        if RE_CRASH.search(rx):
            self._last_crash_time = time.monotonic()
            logger.warning(self.prefix + 'crash detected: %s', rx)

        # always log errors, warnings, etc (ANSI is stripped, so match the ESP log W/E prefix too)
        words = ('shutdown', 'error', 'warn', 'disabled', 'enabled', 'failed', 'reset', 'boot', 'backtrace',
                 'exception')
        if self.verbose:
            print(self.prefix + rx, flush=True)
        elif any(w in rx for w in words) or rx.startswith(('W (', 'E (')):
            logger.warning(self.prefix + 'Ser: %s', rx)

        m = RE_PWM.search(rx)
        if m:
            d = m.groupdict()
            s = PwmState(d['mode'] == 'CCM',
                         pwm_ctrl=int(d['ctrl']),
                         pwm_sync=int(d['sync']),
                         pwm_sync_max=int(d['sync_max']))
            self.wifi_rssi = int(d.get('rssi', 0))
            self.temperatures = [float(d.get('tmp_ntc', nan)), float(d.get('tmp_mcu', nan))]
            self.voltages = [float(d.get('vin', nan)), float(d.get('vout', nan))]
            self._last_telemetry_t = time.monotonic()

            if self.pwm_state != s:
                self.pwm_state = s
            else:
                return

        if 'ina22x' in rx and 'timeout' in rx:
            return

        self.ser_tail.append(rx)
        self.on_message and self.on_message(rx)

        logger.debug('  %s  FUGU: %s', self.prefix, rx)

    def telemetry_age(self):
        """Seconds since the last parsed PWM status line, inf if we never saw one.

        pwm_state/wifi_rssi/temperatures/voltages are only refreshed by that line, so they
        all freeze at their last value when the link wedges or the reader thread dies.
        """
        if self._last_telemetry_t is None:
            return float('inf')
        return time.monotonic() - self._last_telemetry_t

    def telemetry_fresh(self, max_age=None):
        """False if the cached device state is stale (or we never had any).

        Callers that decide something from pwm_state/wifi_rssi/voltages must gate on this,
        otherwise a dead link reads as 'everything still fine' -- the checks would pass
        precisely when we've gone blind. Never returns True on a failure to evaluate.
        """
        age = self.telemetry_age()
        if not self.console.is_alive():
            logger.error(self.prefix + 'reader thread is dead, telemetry is %.1fs old', age)
            return False
        if age > (self.telemetry_max_age if max_age is None else max_age):
            logger.error(self.prefix + 'stale telemetry: last status line %.1fs ago', age)
            return False
        return True

    def get_uptime(self):
        """Device uptime in seconds (monotonic since boot, resets only on reboot), or None.
        Polls the `uptime` command; works on any transport."""
        for l in self.query('uptime'):
            if m := re.search(r'Uptime:\s*(\d+)\s*s', l):
                return int(m.group(1))
        return None

    def get_app_info(self):
        """Running app description from the `uptime` command's App line, as
        {project, version, built, idf}, or None. (esp_app_get_description())"""
        for l in self.query('uptime'):
            if m := re.search(r'App:\s*(\S+)\s+(\S+)\s+\(built (.+), IDF (.+)\)', l):
                return dict(project=m.group(1), version=m.group(2), built=m.group(3), idf=m.group(4))
        return None

    def crash_mark(self):
        """Snapshot (monotonic_now, uptime_s) to pass back to crashed():
        `m = dev.crash_mark(); ...; if dev.crashed(*m): ...`."""
        return time.monotonic(), self.get_uptime()

    def has_crashed(self, reset=False):
        """True if a panic/crash marker was seen since the console opened (or since the last
        reset=True). Serial only: over telnet/BLE/MQTT the panic output and link are lost, so a
        crash can't be observed here — use has_rebooted() instead. Even on serial, native USB-CDC
        (usb_serial_jtag) re-enumerates on the reboot that follows a panic, so the dump can be cut
        off and the reader stop; a hardware UART bridge is more reliable."""
        if not isinstance(self.transport, SerialTransport):
            raise RuntimeError("has_crashed requires a serial transport; use has_rebooted()")
        crashed = self._last_crash_time is not None and self._last_crash_time >= self._crash_baseline_time
        if reset:
            self._crash_baseline_time = time.monotonic()
        return crashed

    def has_rebooted(self, reset=False):
        """True if the device rebooted since the console opened (or since the last reset=True).
        Polls `uptime` and flags a reboot when it regresses; works on any transport. Call
        periodically so a reboot is caught before uptime climbs back past the previous reading."""
        up = self.get_uptime()
        if up is not None:
            if self._last_uptime is not None and up < self._last_uptime:
                self._rebooted = True
                # a reboot may have switched the gate driver (ledc<->mcpwm) -> re-query
                self._pwm_max = self._pwm_ctrl_max = None
                logger.warning(self.prefix + 'reboot detected (uptime %ds -> %ds)', self._last_uptime, up)
            self._last_uptime = up
        rebooted = self._rebooted
        if reset:
            self._rebooted = False
        return rebooted

    def crashed_since(self, since=None, baseline_uptime=None, margin=5.0):
        """Stateless crash/reboot query against a caller-held baseline (see crash_mark()).
        `since`: a monotonic timestamp — True if a panic marker was logged after it (serial only;
        silently skipped on other transports). `baseline_uptime`: a device uptime read earlier —
        True if the device has since rebooted, i.e. the current uptime is less than baseline_uptime
        plus the wall time elapsed since `since` (so a late check still catches it). Either argument
        may be None to skip that check."""
        if since is not None and isinstance(self.transport, SerialTransport):
            if self._last_crash_time is not None and self._last_crash_time > since:
                return True
        if baseline_uptime is not None:
            up = self.get_uptime()
            if up is not None:
                elapsed = (time.monotonic() - since) if since is not None else 0.0
                if up + margin < baseline_uptime + elapsed:
                    return True
        return False

    def get_conf_value(self, file, key):
        self.command_ack(f"get-config {file} {key}")
        rex = re.compile(rf"(.+: )?Conf '/littlefs/conf/{file}:{key}' = '(.*)'")
        for l in reversed(self.ser_tail):
            if m := rex.match(l):
                return m.group(2)  # group(1) is the optional log prefix; group(2) is the value
        return None

    def manual_pwm(self, en=True):
        if en:
            d = max(1, self.pwm_state.pwm_ctrl)
            # d += -1 if d > 2 else + 1
            # self.set_D(d)
            self.write('dc %d\n' % d)
        else:
            self.write("mppt\n")

    # --- pwm ------------------------------------------------------------------------------------

    @property
    def pwm_max(self) -> int:
        """PWM resolution of the active gate driver, queried once from the device."""
        if self._pwm_max is None:
            self.query_pwm_limits()
        if self._pwm_max is None:
            raise RuntimeError('device did not report pwmMax (pwm-dump unsupported?), '
                               'cannot convert duty ratios; use set_pwm() with raw counts')
        return self._pwm_max

    @property
    def pwm_ctrl_max(self) -> Optional[int]:
        if self._pwm_ctrl_max is None and self._pwm_max is None:
            self.query_pwm_limits()
        return self._pwm_ctrl_max

    def query_pwm_limits(self):
        """Populate pwm_max/pwm_freq from `pwm-dump` and pwm_ctrl_max (the `dc` clamp
        limit) from an out-of-range `dc` probe. Raises if neither yields anything --
        never silently assumes a resolution."""
        try:
            for l in self.query('pwm-dump'):
                if m := re.search(r'freq=(\d+) pwmMax=(\d+)', l):
                    self.pwm_freq = int(m.group(1))
                    self._pwm_max = int(m.group(2))
        except Exception as e:
            logger.warning(self.prefix + 'pwm-dump failed (%s), probing dc limit', e)
        # rejected before any mode/duty change, so this has no side effect
        for l in self.console.command('dc 999999999'):
            if m := re.search(r'out of range \[0,(\d+)\]', l):
                self._pwm_ctrl_max = int(m.group(1))
        # the clamp limit is mandatory: without it set_duty() could send counts the
        # firmware silently rejects (never degrade to an unverified state)
        if self._pwm_ctrl_max is None:
            had_max = self._pwm_max is not None
            self._pwm_max = None
            raise RuntimeError('cannot determine PWM limits (dc probe failed%s)'
                               % ('' if had_max else ', pwm-dump too'))
        logger.debug(self.prefix + 'pwm limits: pwm_max=%s pwm_ctrl_max=%s freq=%s',
                     self._pwm_max, self._pwm_ctrl_max, self.pwm_freq)

    def set_pwm(self, pwm_cnt, step_wait=0.05) -> int:
        """Set raw HS on-count (`dc` units, driver-resolution dependent), ramping in steps.
        Returns the count sent."""
        pwm_cnt = int(pwm_cnt)
        # ~0.5% of range per step, same fade rate as the tuned 10/2048
        max_step = max(10, round(self._pwm_max * 10 / 2048)) if self._pwm_max else 10

        pwm_ctrl = self.pwm_state.pwm_ctrl

        while pwm_ctrl != pwm_cnt:
            delta = pwm_cnt - pwm_ctrl
            # dont fade if target is 0
            if pwm_cnt != 0 and abs(delta) > max_step:
                delta = max_step * delta / abs(delta)
            pwm_ctrl += delta
            self.transport.write(b'dc %d\n' % pwm_ctrl)
            time.sleep(step_wait / 4 if delta < 0 else step_wait)

        logger.debug('Set pwm_cnt = %d', pwm_cnt)
        return pwm_cnt

    def set_D(self, pwm_cnt, step_wait=0.05) -> int:
        # deprecated: the name suggests a duty ratio but the unit is raw counts
        if not FuguDevice._warned_set_d:
            FuguDevice._warned_set_d = True
            logger.warning('set_D() is deprecated, use set_pwm() (raw counts) or set_duty() (0..1)')
        return self.set_pwm(pwm_cnt, step_wait)

    def set_duty(self, d: float, step_wait=0.05) -> int:
        """Set duty ratio 0..1, converted with the device-reported resolution.
        Returns the raw count sent (compare pwm_state.pwm_ctrl against this)."""
        assert 0 <= d <= 1, d
        cnt = round(d * self.pwm_max)
        if cnt > self._pwm_ctrl_max:  # always known when pwm_max is (see query_pwm_limits)
            logger.warning(self.prefix + 'duty %.3f clamped to pwm_ctrl_max=%d', d, self._pwm_ctrl_max)
            cnt = self._pwm_ctrl_max
        return self.set_pwm(cnt, step_wait)

    def get_duty(self) -> float:
        return self.pwm_state.pwm_ctrl / self.pwm_max

    def wifi_power(self, on, minutes=None):
        # not acked: turning Wi-Fi off drops telnet/BLE, so an OK marker may never arrive.
        # `minutes` re-enables after a timeout (keeps the stored SSID); bare off forgets it.
        if on:
            self.write('wifi on\n')
        elif minutes:
            self.write('wifi off %d\n' % int(minutes))
        else:
            self.write('wifi off\n')

    def wifi_add(self, ssid, password):
        self.command_ack('wifi-add %s:%s' % (ssid, password))

    def write(self, cmd: str):
        self.transport.write(cmd.encode('utf-8'))

    def command_ack(self, cmd: str):
        reply = self.console.command(cmd.strip())
        if reply.ok:
            return True
        if reply.timed_out and not reply:
            logger.info(self.prefix + 'Never received anything')
        for l in reply:
            logger.warning(self.prefix + 'Ser: %s', l)
        raise Exception(f"unexpected serial response for command '{cmd}'")

    def sync_rect_enable(self, state: Union[bool, Literal['forced']]):
        if state == 'forced':
            self.command_ack('sync forced')
        else:
            self.command_ack('sync ' + str(int(state)))

    def backflow_enable(self, enable):
        # bf/panel switch (output->input). requires manual PWM and a configured backflow switch.
        self.command_ack('bf 1' if enable else 'bf 0')

    # the "ideal diode" is the backflow switch; kept as an alias for older callers
    ideal_diode_enable = backflow_enable

    def query(self, cmd: str, timeout: float = 4.0):
        """Run a command and return its Reply (the lines before the OK marker). Raises on
        rejection/timeout, like command_ack."""
        reply = self.console.command(cmd.strip(), timeout=timeout)
        if reply.ok:
            return reply
        for l in reply:
            logger.warning(self.prefix + 'Ser: %s', l)
        raise Exception(f"unexpected serial response for command '{cmd}'")

    # --- tracker / converter -------------------------------------------------------------------

    def sweep(self):
        self.command_ack('sweep')

    def mppt_enable(self):
        # leave manual PWM, resume MPP tracking
        self.command_ack('mppt')

    def set_speed(self, scale: float):
        self.command_ack('speed %g' % scale)

    def short_ls(self):
        # boost topology with Vin~0 only
        self.command_ack('short-ls')

    # --- charger limits (runtime only; use set_config to persist) ------------------------------

    def set_vbat_max(self, volts: float):
        self.command_ack('vset %g' % volts)

    def set_ibat_lim(self, amps: float):
        self.command_ack('iset %g' % amps)

    def set_vout(self, volts: float, tol_v=0.5, timeout=20.0, hold=1.0, poll=0.25):
        """Set the output-voltage setpoint and WAIT until the measured Vout actually holds it.

        Uses the firmware's own regulator (`vset` -> charger Vbat_max). Do NOT drive output
        voltage by stepping duty from the host: a console round trip is orders of magnitude
        slower than the converter's control loop, and a host loop fights the regulator it is
        trying to help.

        Returns the measured Vout. Raises rather than returning a voltage it did not see:

        * TimeoutError if Vout never enters the band, or does not stay in it for `hold`
          seconds -- reported WITH the measured value, so the caller can tell "regulator is
          still ramping" from "this setpoint is unreachable".
        * ConnectionError if telemetry goes stale, because a frozen `voltages` reads as a
          perfectly settled rail exactly when we have gone blind. Absence of evidence must
          not encode a settled output.

        `tol_v` is the acceptance band, not a controller gain -- nothing here regulates.
        """
        self.set_vbat_max(volts)
        t0 = time.monotonic()
        in_band_since = None
        last = nan
        while time.monotonic() - t0 < timeout:
            if not self.telemetry_fresh():
                raise ConnectionError(
                    '%sset_vout(%.3f): telemetry went stale (%.1fs old) -- the measured output '
                    'is unknown, not settled' % (self.prefix, volts, self.telemetry_age()))
            last = self.voltages[1] if len(self.voltages) > 1 else nan
            if last == last and abs(last - volts) <= tol_v:
                in_band_since = in_band_since or time.monotonic()
                if time.monotonic() - in_band_since >= hold:
                    logger.debug(self.prefix + 'set_vout(%.3f) -> %.3fV', volts, last)
                    return last
            else:
                in_band_since = None      # must hold, not just touch: overshoot is not settled
            time.sleep(poll)
        raise TimeoutError(
            '%sset_vout(%.3f): Vout=%.3fV after %.0fs, outside +/-%.3fV%s'
            % (self.prefix, volts, last, timeout, tol_v,
               ' (reached the band but never held it)' if in_band_since else ''))

    # --- misc actuators ------------------------------------------------------------------------

    def set_fan(self, percent: float):
        self.command_ack('fan %g' % percent)

    def set_led(self, color):
        # hex 'RRGGBB' or short 'RGB'
        self.command_ack('led %s' % color)

    # --- config files --------------------------------------------------------------------------

    def set_config(self, file, key, value):
        self.command_ack('set-config %s %s %s' % (file, key, value))

    def del_config(self, file, key):
        self.command_ack('del-config %s %s' % (file, key))

    # --- system / diagnostics ------------------------------------------------------------------

    def restart(self):
        self.write('restart\n')  # device reboots; no OK marker comes back

    def adc_restart(self):
        self.command_ack('adc-restart')

    def adc_reset(self):
        self.command_ack('adc-reset')

    def reset_lag(self):
        self.command_ack('reset-lag')

    def scan_i2c(self):
        self.command_ack('scan-i2c')

    def rt_stats(self):
        # the per-task stats are printed asynchronously (sampled over ~1 s) after the OK marker,
        # so they surface via the _on_line tap, not the returned ack.
        self.command_ack('rt-stats')

    def set_hostname(self, name):
        self.command_ack('hostname %s' % name)

    def get_hostname(self):
        for l in self.query('hostname'):
            if m := re.search(r'Hostname:\s*(\S+)', l):
                return m.group(1)
        return None

    def get_ip(self):
        for l in self.query('ip'):
            if m := re.search(r'Local IP Address:\s*(\S+)', l):
                return m.group(1)
        return None

    def get_mem(self):
        keys = {'Total heap': 'heap_total', 'Free heap': 'heap_free',
                'Total PSRAM': 'psram_total', 'Free PSRAM': 'psram_free'}
        mem = {}
        for l in self.query('mem'):
            if m := re.search(r'(Total heap|Free heap|Total PSRAM|Free PSRAM):\s*(\d+)', l):
                mem[keys[m.group(1)]] = int(m.group(2))
        return mem

    def get_sensor_avg(self):
        """Compact EWM averages as a dict, e.g. {'vin': .., 'vout': .., 'iout': ..}."""
        for l in self.query('sensor avg'):
            if 'sens:' in l:
                return {k: float(v) for k, v in re.findall(r'(\w+)=(nan|-?\d+\.?\d*(?:e-?\d+)?)', l)}
        return {}

    def get_sensors(self) -> str:
        """Full per-sensor dump as raw text."""
        return self.query('sensor').text

    # --- wired sync ----------------------------------------------------------------------------

    def wsync_edge_rate(self) -> Optional[float]:
        """Edge rate on the wired-sync pin (`board.conf::pwm_sync_pin`) in Hz, from the `wsync`
        PCNT diagnostic. On a follower that is the leader's incoming pulse, on a leader its own
        outgoing one.

        None means *not measured* (sync_role=none, firmware built without WSYNC, saturated
        count, command rejected) -- distinct from 0.0, which means the counter ran and the pin
        was silent."""
        try:
            reply = self.query('wsync')
        except Exception as e:
            logger.warning(self.prefix + 'wsync failed: %s', e)
            return None
        for l in reply:
            if 'no edge counter' in l:
                return None
            if m := re.search(r'wsync edges:\s*(\d+)\s+in\s+(\d+)\s*ms', l):
                n, ms = int(m.group(1)), int(m.group(2))
                return n * 1000.0 / ms if ms else None
        return None

    def wsync_status(self, tol=0.05) -> dict:
        """Wired-sync health of *this* device as
        {synced, role, rate_hz, expected_hz, reason}.

        `synced` is tri-state and never guesses in favour of health:
          True  -- role is `follower` and the sync pin carries the leader's pulse at this
                   converter's own pwm_freq (within `tol`, default 5%).
          False -- the pin is measurably wrong: no edges, or a rate that isn't pwm_freq
                   (leader off, wire broken, mismatched pwm_freq).
          None  -- could not be evaluated: role unknown or not `follower`, no pwm_freq, no edge
                   counter, command failed. `reason` says which.

        Caveat: this proves the leader's pulse arrives at the expected rate, which is what the
        follower's MCPWM timer hardware-reloads on -- it does not read back the timer phase. A
        confirmed lock (switch nodes stationary, no beat) still needs a scope."""
        out: dict = dict(synced=None, role=None, rate_hz=None, expected_hz=None, reason='')

        try:
            # absent key reads back as '', i.e. the firmware default
            out['role'] = self.get_conf_value('converter.conf', 'sync_role') or 'none'
        except Exception as e:
            out['reason'] = 'sync_role unreadable: %s' % e
            return out
        if out['role'] != 'follower':
            out['reason'] = 'sync_role=%s, not a follower' % out['role']
            return out

        expected = self.pwm_freq
        if expected is None:
            try:
                self.query_pwm_limits()
                expected = self.pwm_freq
            except Exception as e:
                logger.warning(self.prefix + 'pwm limits unavailable: %s', e)
        if expected is None:
            try:
                expected = float(self.get_conf_value('converter.conf', 'pwm_freq') or 0) or None
            except Exception:
                expected = None
        if not expected:
            out['reason'] = 'pwm_freq unknown, cannot judge the edge rate'
            return out
        out['expected_hz'] = float(expected)

        rate = self.wsync_edge_rate()
        out['rate_hz'] = rate
        if rate is None:
            out['reason'] = 'no edge count (WSYNC off, or the diagnostic failed)'
            return out

        # the measurement window is a vTaskDelay (1 ms tick) that a busy device can overrun, so
        # the rate reads low under load; tol covers that, not a real frequency offset
        dev = (rate - expected) / expected
        out['synced'] = abs(dev) <= tol
        if out['synced']:
            out['reason'] = 'sync pulse at %.0f Hz (pwm_freq %.0f, %+.1f%%)' % (rate, expected, 100 * dev)
        elif rate == 0:
            out['reason'] = 'no sync pulse on the wire'
        else:
            out['reason'] = 'edge rate %.0f Hz != pwm_freq %.0f Hz (%+.1f%%)' % (rate, expected, 100 * dev)
        return out

    def wsync_synced(self, tol=0.05) -> Optional[bool]:
        """Tri-state shorthand for wsync_status()['synced'] -- None = could not verify."""
        return self.wsync_status(tol)['synced']

    # --- services ------------------------------------------------------------------------------

    def svc_list(self):
        """Parse `svc list` into a list of {name, state, log, enabled, detail} dicts."""
        out = []
        for l in self.query('svc'):
            parts = l.split(None, 4)
            if len(parts) < 4 or parts[3] not in ('yes', 'no'):
                continue  # skip the header and any interleaved status lines
            out.append({'name': parts[0], 'state': parts[1], 'log': parts[2],
                        'enabled': parts[3] == 'yes', 'detail': parts[4] if len(parts) > 4 else ''})
        return out

    def svc(self, action, name=None, level=None):
        """svc on|off|restart|rs <name>, svc log <name> <level>, or svc list (-> svc_list())."""
        if action == 'list':
            return self.svc_list()
        if action == 'log':
            return self.command_ack('svc log %s %s' % (name, level))
        return self.command_ack('svc %s %s' % (action, name))

    def __iadd__(self, i):
        self.set_pwm(self.pwm_state.pwm_ctrl + i)
        return self

    def is_connected(self):
        return self.console.is_alive()

    def power_loop_rig_sequence_buck(dev, target_duty=0.4):
        dev.wifi_power(False)

        dev.manual_pwm()
        dev.sync_rect_enable(True)  # before shutdown sequence, make sure to disable forced PWM
        dev.set_pwm(1)  # minimum on-time, intentionally a raw count
        dev.sync_rect_enable(False)
        dev.ideal_diode_enable(False)
        while dev.voltages[0] > 80 or dev.voltages[0] < 68:
            print(dev.prefix, 'waiting for input voltage to converge', dev.voltages)
            time.sleep(1)

        dev.set_duty(0.195)
        dev.sync_rect_enable(True)
        dev.ideal_diode_enable(True)
        time.sleep(.4)
        dev.set_duty(0.293)
        time.sleep(.2)
        dev.set_duty(target_duty, step_wait=0.15)
        dev.sync_rect_enable('forced')


if __name__ == '__main__':
    dev = FuguDevice(ip='192.168.4.2')

    dev.command_ack("reset")

    sys.exit(0)

    duty = 0.146
    while True:
        dev.set_duty(duty)
        time.sleep(.2)

        if duty == 0.732:
            break
        duty = min(duty * 1.1, 0.732)
