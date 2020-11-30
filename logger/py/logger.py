from rj.StatusFrameHolder import *
from rj.InitializeStatusFrame import *
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

f = None

try:
    while True:
        data, addr = sock.recvfrom(255)
        buf = bytearray(data)
        sfc = StatusFrameHolder.GetRootAsStatusFrameHolder(data, 4)
        if sfc.StatusFrameType() == 4:
            tab = sfc.StatusFrame()
            init = InitializeStatusFrame.GetRootAsInitializeStatusFrame(tab.Bytes, tab.Pos)
            f = open(f"runtime.{init.Title()}.dat")
            del tab
            del init

        print(struct.unpack("<I", data[:4])[0] == len(data)-4, sfc.StatusFrameType(), sfc.MonotonicTime())
        if f is not None:
            f.write(data)
finally:
    if f is not None:
        f.close()


