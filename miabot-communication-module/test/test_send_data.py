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

while True:
    # print("stop wheels!")
    # ser.write(stopWheels)
    # sleep(1)
    # print("start wheels fwd 30")
    # ser.write(moveForward30)
    # sleep(3)
    print("move fwd")
    ser.write(moveFwd30)
    sleep(5)
    print("move fwd")
    ser.write(b"[o1]\n")
    sleep(5)
    print("move fwd")
    ser.write(b"[o15]\n")
    sleep(5)
    print("move fwd")
    ser.write(b"[o30]\n")
    sleep(5)
    # print("move back")
    # ser.write(moveBack30)
    # sleep(5)
    # print("move left")
    # ser.write(moveLeft30)
    # sleep(5)
    # print("move right")
    # ser.write(moveRight30)
    # sleep(5)


