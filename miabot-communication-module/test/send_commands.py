"""
Send commands to Miabot Pro tests
add tests like
set speed to 30 and ride for 5 seconds
set speed to 50 and ride for 5 secs
…
"""

from time import sleep
import serial

stopWheels = "[s]\n"
moveForward30 = "[=<30l>,<30r>]\n"
stepLen90Deg = "[d>1334]\n"
stepLen100mm = "[d^2533]\n"

stepFwd = "[^]\n"
stepRight = "[>]\n"

moveFwd30 = "[o99]\n"
moveBack30 = "[p40]\n"
moveLeft30 = "[m50]\n"
moveRight30 = "[n60]\n"
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

def write_read_data(inp: str) -> None:
    
    ser.write(str.encode(inp))
    print(ser.readall().decode())

    return

def circle_steps() -> None:
    write_read_data(stepLen90Deg)
    write_read_data(stepLen100mm)
    write_read_data(stepFwd)
    sleep(4)
    write_read_data(stepRight)
    sleep(4)
    write_read_data(stepFwd)
    sleep(4)
    write_read_data(stepRight)
    sleep(4)
    write_read_data(stepFwd)
    sleep(4)
    write_read_data(stepRight)
    sleep(4)
    write_read_data(stepFwd)
    sleep(4)
    write_read_data(stepRight)
    
    return

def circle_():
    write_read_data("[=40l,20r]")
    sleep(20)
    write_read_data(stopWheels)
    return

while True:
    print("terminal ready")
    inp = input()
    match inp:
        case "circle_steps":
            circle_steps()
        case "circle_":
            circle_()
        case default:
            inp += '\n'
            write_read_data(inp)
