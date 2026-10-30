"""树莓派USB串口链路；pip install pyserial。默认仅发送停车命令。"""
import struct
import time
import serial

def crc16(data):
    c = 0xffff
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xa001 if c & 1 else c >> 1
    return c

class Link:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.1)
        self.sequence = 0

    def send(self, command, a=0.0, b=0.0):
        payload = struct.pack('<BBBBHff', 0xa5, 0x5a, 1, command,
                              self.sequence, a, b)
        self.sequence = (self.sequence + 1) & 0xffff
        frame = payload + struct.pack('<H', crc16(payload))
        if self.port.write(frame) != len(frame):
            raise IOError('串口发送不完整')

    def enable(self): self.send(1)
    def drive(self, v, omega): self.send(2, v, omega)
    def arm(self, preset): self.send(3, preset)
    def heartbeat(self): self.send(4)
    def stop(self): self.send(0)

if __name__ == '__main__':
    link = Link()
    try:
        for _ in range(10):
            link.stop()
            time.sleep(0.05)
    finally:
        link.stop()
        link.port.close()
