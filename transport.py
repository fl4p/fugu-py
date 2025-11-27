import glob
import socket
from typing import Optional

import serial # pyserial

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
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(4)

    def open(self):
        logger.info('connecting to %s:%u', *self.addr)
        self.sock.connect(self.addr)

    def close(self):
        self.sock.close()

    def read(self):
        return self.sock.recv(1024)

    def write(self, data):
        return self.sock.send(data)
