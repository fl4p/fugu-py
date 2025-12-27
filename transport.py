import glob
import socket
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

    def __init__(self, port):
        self.port = port
        self.ser: Optional[serial.Serial] = None

    def open(self):
        if self.ser and self.ser.is_open:
            return
        port = self.port
        if '*' in port:
            port = glob.glob(port)[0]
        logger.info(f'opening serial port {port}')
        self.ser = serial.Serial(port, baudrate=115200)

    def write(self, data: bytes):
        self.ser.write(data)

    def read(self) -> Optional[bytes]:
        if self.ser.is_open and self.ser.readable():
            return self.ser.readline()
        return None


class SocketTransport(Transport):
    DEFAULT_PORT = 23  # telnet

    def __init__(self, ip, port=DEFAULT_PORT, is_telnet=True):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(6)
        self.is_telnet = is_telnet
        self.t_last_comm = time.time()


    def open(self):
        logger.info('connecting to %s:%u', *self.addr)
        self.sock.connect(self.addr)

    def close(self):
        self.sock.close()

    def read(self):
        try:
            r = self.sock.recv(1024)
            if r:
                self.t_last_comm = time.time()
            if time.time() - self.t_last_comm > 1:
                # check conn health
                if self.is_telnet:
                    self.write(bytes([255, 241]))  # send telnet NOP to probe conn TODO Are you there 246 ?
        except BrokenPipeError:
            print(self.sock, 'BrokenPipeError')
            self.close()
        return r

    def write(self, data):
        i = self.sock.send(data)
        if i > 0:
            self.t_last_comm = time.time()
        return i

    def check_connection(self) -> bool:
        if time.time() - self.t_last_comm < 2:
            return True

        try:
            # this will try to read bytes without blocking and also without removing them from buffer (peek only)
            data = self.sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
            if len(data) == 0:
                return True
        except ConnectionResetError as e:
            print(type(e), e)
            self.close()
            return False  # socket was closed for some other reason
        except BlockingIOError:
            return True  # socket is open and reading from it would block
        except OSError as e:
            print(type(e), e)
            return False  # 'Bad file descriptor' (socket closed locally)
        except Exception as e:
            print("unexpected exception when checking if a socket is closed", type(e), e)
            return True
        return True
