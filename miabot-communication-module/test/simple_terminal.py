"""
Send commands to Miabot Pro tests
"""

from time import sleep
import serial

stopWheels = b"[s]\n"
moveForward30 = b"[=<30l>,<30r>]\n"
moveFwd30 = b"[o99]\n"
moveBack30 = b"[p40]\n"
moveLeft30 = b"[m50]\n"
moveRight30 = b"[n60]\n"
defaultMiabotPort = "/dev/ttyS0"
defaultBaudrate = 115200

ser = serial.Serial(
    port=defaultMiabotPort,
    baudrate=defaultBaudrate,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)


print(type(moveBack30))

while True:
    print("terminal ready")
    inp = input()
    inp += '\n'
    ser.write(str.encode(inp))
    print(ser.readall().decode())
