"""
Here will be our description
There will be a lot to do here
- move to another modules
- typing!
- ...
"""

from json import dumps
from random import randint
from time import sleep
import signal

import serial

from src.hokuyo_lib.driver import hokuyo
from src.hokuyo_lib.tools import serial_port

UART_PORT = '/dev/ttyACM0'
UART_SPEED = 19200

defaultMiabotPort = "/dev/ttyS0"
defaultBaudrate = 115200

rotate_right = b"[n90]\n"
step_len = b"[d^4000]\n"
step_fwd = b"[^]\n"
stop_cmd = b"[s]\n"

Sentry = True

# Create a Signal Handler for Signals.SIGINT:  CTRL + C 
def SignalHandler_SIGINT(SignalNumber,Frame):
   global Sentry 
   Sentry = False
   
signal.signal(signal.SIGINT,SignalHandler_SIGINT) #regsiter signal with handler

ser = serial.Serial(
    port=defaultMiabotPort,
    baudrate=defaultBaudrate,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

front_angle: int = 30
turn_while_obstacle_is_closed_than: int = 200       # 200 is equal to 20 cm
max_allowed_speed: int = 80
def speed_msg(
        turn_while_obstacle_is_closed_than: int,
        min_dist: int
    ) -> str:

    set_speed = min(max_allowed_speed, int(min_dist-turn_while_obstacle_is_closed_than))
    print(f"set_speed: {set_speed}")
    return f"[=<{set_speed}l>,<{set_speed}r>]\n".encode("utf-8")

if __name__ == '__main__':
    laser_serial = serial.Serial(
        port=UART_PORT, baudrate=UART_SPEED, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)

    # print(laser.set_high_sensitive())
    ser.write(step_len)

    while Sentry:
        laser.laser_on()
        data: dict = laser.get_single_scan()
        laser.reset()
        fixed_data = {}
        for rev_ang in data:
            fixed_data[-float(rev_ang)] = data[rev_ang]

        front_obtacle_distances: list = [
            fixed_data[i] for i in fixed_data if abs(int(float(i))) <= front_angle
        ]
        print(front_obtacle_distances)
        # average
        # front_obtacle_distance: int = sum(front_obtacle_distances) / len(front_obtacle_distances)
        # minimum
        # front_obtacle_distance: int = min(front_obtacle_distances)
        # some minimum from 10% minimum
        front_obtacle_distances = list(filter(lambda num: num != 0, front_obtacle_distances))
        front_obtacle_distances.sort()
        try:
            front_obtacle_distance: int = sum(front_obtacle_distances[
                :len(front_obtacle_distances)//10]) / (len(front_obtacle_distances)//10)
        except ZeroDivisionError:
            print("no valid measures")

        if front_obtacle_distance < turn_while_obstacle_is_closed_than:
            print("I will change my direction, measured dist:",
                  front_obtacle_distance)
            ser.write(stop_cmd)
            if randint(0, 2**16) % 2 == 0:
                ser.write(b"[=<10l>,<-10r>]\n")
            else:
                ser.write(b"[=<-10l>,<10r>]\n")
            # ser.write(rotate_right)
            sleep(0.3)
        else:
            print("I will move forward, measured dist:", front_obtacle_distance)
            # ser.write(step_fwd)
            ser.write(stop_cmd)
            ser.write(speed_msg(
                turn_while_obstacle_is_closed_than,
                front_obtacle_distance
            ))

        laser.laser_off()
        # print(dumps(fixed_data, indent=4))
        # sleep(0.2)

    print("I will stop!")
    ser.write(stop_cmd)
