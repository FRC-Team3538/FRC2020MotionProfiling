from rj.StatusFrameHolder import *
import socket
import json
import time
import struct

UDP_IP_ADDRESS = '0.0.0.0'
UDP_PORT_NO = 5801

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))
sock.setblocking(0)
sock.settimeout(0.1)

with open("runtime.log", "wb") as f:
    while True:
        data, addr = sock.recvfrom(255)
        f.write(data)
        buf = bytearray(data)
        sfc = StatusFrameHolder.GetRootAsStatusFrameHolder(data, 4)
        print(struct.unpack("<I", data[:4])[0] == len(data)-4, sfc.StatusFrameType(), sfc.MonotonicTime())

