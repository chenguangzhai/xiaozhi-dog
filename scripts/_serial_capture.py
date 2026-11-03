import sys
import time

import serial

PORT = "COM9"
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

p = serial.Serial(PORT, 115200, timeout=0.2, dsrdtr=False, rtscts=False)
# USB-Serial-JTAG 的 idle 状态是 DTR=0 / RTS=0，先摆正，避免开端口时把芯片按在复位里
p.dtr = False
p.rts = False
time.sleep(0.3)

# 硬复位：RTS 拉低 EN -> 松开即启动
p.rts = True
time.sleep(0.15)
p.rts = False

chunks = []
t0 = time.time()
while time.time() - t0 < DURATION:
    data = p.read(8192)
    if data:
        chunks.append(data)
p.close()

sys.stdout.buffer.write(b"".join(chunks))
