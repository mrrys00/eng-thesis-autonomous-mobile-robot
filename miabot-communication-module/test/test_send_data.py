"""
Send commands to Miabot Pro tests
"""

from time import sleep
import serial

stopWheels = b"[s]\n"
moveForward30 = b"[=<30l>,<30r>]\n"
defaultMiabotPort = "/dev/ttys0"
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
    print("stop wheels!")
    ser.write(stopWheels)
    sleep(1)
    print("start wheels fwd 30")
    ser.write(moveForward30)
    sleep(3)
