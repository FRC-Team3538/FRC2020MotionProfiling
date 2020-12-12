from rj.StatusFrameHolder import *
from rj.InitializeStatusFrame import *
import socket
import json
import time
import struct
import sys

ROBOT_LOGGER_ADDRESS = (sys.argv[1], int(sys.argv[2])) if len(sys.argv) == 3 else ("10.83.32.2", 3538)

f = None
init = None

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(0.1)

    try:
        sock.sendto(b"Hi", ROBOT_LOGGER_ADDRESS)
        while True:
            data = sock.recv(4096)
            sfc = StatusFrameHolder.GetRootAsStatusFrameHolder(data, 4)
            if sfc.StatusFrameType() == 4:
                tab = sfc.StatusFrame()
                init = InitializeStatusFrame()
                init.Init(
                    tab.Bytes, tab.Pos
                )
                break
        title = init.Title().decode('utf-8')[:-1].replace(' ', '.').replace(':', '')
        with open(f"logs/runtime.{title}.dat", "wb") as f:
            print(f"Starting log: runtime.{title}.dat")
            while True:
                data, addr = sock.recvfrom(4096)
                buf = bytearray(data)
                f.write(data)
    except Exception as e:
        print(f"Received an Exception: {e}")
        raise e

print("Exiting...")
