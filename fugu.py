import collections
import re
import time
from threading import Thread
from typing import Optional, Literal, Union

from math import nan

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
    @staticmethod
    def get_default_serial_port():
        import socket
        return '/dev/ttyACM1' if socket.gethostname() == 'rpi' else '/dev/cu.usbmodem*'

    def __init__(self, transport: Transport = None, ip=None, prefix='', block=True):
        self.pwm_state = PwmState(None, 0, 0, 0)
        self.wifi_rssi = 0

        if ip:
            assert transport is None
            transport = SocketTransport(ip, 23)
        elif transport is None:
            transport = SerialTransport(FuguDevice.get_default_serial_port())

        self.ser_deque = collections.deque()
        self.ser_tail = collections.deque(maxlen=20)
        self.prefix = prefix

        self.transport = transport

        self.transport.open()

        self.temperatures = []

        self.is_open = True
        self._rx_thread = Thread(target=self._recv_loop, daemon=True)
        self._rx_thread.start()

        self.verbose = False
        self.on_message = None

        if block:
            while self.pwm_state.ccm is None:
                time.sleep(0.1)

    def open(self):
        assert not self.is_open
        raise NotImplementedError()

    def wait_for_pwm_state(self):
        assert self._rx_thread.is_alive()
        self.pwm_state = PwmState(None, 0, 0, 0)
        while self.pwm_state.ccm is None:
            time.sleep(0.1)

    def close(self, close_transport=True):
        self.is_open = False
        self.pwm_state = PwmState(None, 0, 0, 0)
        if self._rx_thread.is_alive():
            self._rx_thread.join()
        if close_transport:
            self.transport.close()

    def _recv_loop(self):
        while self.is_open:
            try:
                rx_b = self.transport.read()
                rx = rx_b.decode('utf-8').strip()
            except TimeoutError as e:
                time.sleep(.2)
                continue
            except UnicodeDecodeError as e:
                print('decode error', e)
                time.sleep(1)
                continue

            if not rx:
                time.sleep(.01)
                continue

            # always log errors, warnings, etc
            words = ('shutdown', 'error', 'warn', 'disabled', 'enabled', 'failed', 'reset', 'boot', 'backtrace',
                     'exception')
            if self.verbose:
                print(self.prefix + rx, flush=True)
            else:
                if (any(map(lambda w: w in rx, words)) or b'\x1b[0;33mW ' in rx_b or b'\x1b[0;33mE ' in rx_b):
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

                if self.pwm_state != s:
                    self.pwm_state = s
                else:
                    continue

            if 'ina22x' in rx and 'timeout' in rx:
                continue

            self.ser_deque.append(rx)
            self.ser_tail.append(rx)
            self.on_message and self.on_message(rx)

            logger.debug('  %s  FUGU: %s', self.prefix, rx)

    def get_conf_value(self, file, key):
        self.command_ack(f"get-config {file} {key}")
        rex = re.compile(rf".+: Conf '/littlefs/conf/{file}:{key}' = '(.*)'")
        for l in reversed(self.ser_tail):
            if m := rex.match(l):
                return m.group(1)
        return None

    def manual_pwm(self, en=True):
        if en:
            d = max(1, self.pwm_state.pwm_ctrl)
            # d += -1 if d > 2 else + 1
            # self.set_D(d)
            self.write('dc %d\n' % d)
        else:
            self.write("mppt\n")

    def set_D(self, pwm_cnt, step_wait=0.05):
        max_step = 10

        pwm_ctrl = self.pwm_state.pwm_ctrl

        while pwm_ctrl != pwm_cnt:
            delta = pwm_cnt - pwm_ctrl
            # dont fade if target is 0
            if pwm_cnt != 0 and abs(delta) > max_step:
                delta = max_step * delta / abs(delta)
            pwm_ctrl += delta
            self.transport.write(b'dc %d\n' % pwm_ctrl)
            time.sleep(step_wait / 4 if delta < 0 else step_wait)

        # TODO wait?

        # self.transport.send(b'dc %d\n' % pwm_cnt)
        logger.debug('Set pwm_cnt = %d', pwm_cnt)

    def wifi_power(self, on):
        self.transport.write(b'wifi %s\n' % (b'on' if on else b'off'))

    def write(self, cmd: str):
        self.transport.write(cmd.encode('utf-8'))

    def command_ack(self, cmd: str):
        ser_deque = self.ser_deque
        ser_deque.clear()

        self.write(cmd.strip() + '\n')

        l = ""
        ok_resp = "OK: " + cmd.strip()
        for _ in range(1, 20):
            while len(ser_deque):
                l = ser_deque.popleft()
                if ok_resp in l.strip():
                    return True
            time.sleep(0.1)

        if len(ser_deque) == 0:
            logger.info('Never received anything')

        while len(ser_deque):
            logger.warning(self.prefix + 'Ser: %s', ser_deque.popleft())

        raise Exception(f"unexpected serial response '{l}' for command '{cmd}")

        self.transport.write(cmd.encode('utf-8'))

    def sync_rect_enable(self, state: Union[bool, Literal['forced']]):
        if state == 'forced':
            self.command_ack('sync forced')
        else:
            self.command_ack('sync ' + str(int(state)))

    def ideal_diode_enable(self, enable):
        self.command_ack('bf-enable' if enable else 'bf-disable')

    def __iadd__(self, i):
        self.set_D(self.pwm_state.pwm_ctrl + i)
        return self

    def is_connected(self):
        return self._rx_thread.is_alive()

    def power_loop_rig_sequence_buck(dev, target_d=770):
        dev.wifi_power(False)

        dev.manual_pwm()
        dev.sync_rect_enable(True)  # before shutdown sequence, make sure to disable forced PWM
        dev.set_D(1)
        dev.sync_rect_enable(False)
        dev.ideal_diode_enable(False)
        while dev.voltages[0] > 80 or dev.voltages[0] < 70:
            print('waiting for input voltage to converge', dev.voltages)
            time.sleep(1)

        dev.set_D(400)
        dev.sync_rect_enable(True)
        dev.ideal_diode_enable(True)
        time.sleep(1)
        dev.set_D(600)
        time.sleep(.2)
        # dev.set_D(700)
        # time.sleep(1)
        dev.set_D(target_d, step_wait=0.15)
        dev.sync_rect_enable('forced')


if __name__ == '__main__':
    dev = FuguDevice(ip='192.168.178.222')

    pwm_cnt = 300
    while True:
        dev.set_D(pwm_cnt)
        time.sleep(.2)

        if pwm_cnt == 1500:
            break
        pwm_cnt = min(pwm_cnt * 1.1, 1500)
