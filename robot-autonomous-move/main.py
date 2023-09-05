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


import serial

from hokuyo_lib.driver import hokuyo
from hokuyo_lib.tools import serial_port

UART_PORT = '/dev/ttyACM0'
UART_SPEED = 19200

def choose_direction():
    pass

if __name__ == '__main__':
    laser_serial = serial.Serial(port=UART_PORT, baudrate=UART_SPEED, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)

    front_angle: int = 10
    turn_while_obstacle_is_closed_than: int = 200       # 200 is equal to 20 cm

    print(laser.set_high_sensitive())

    while True:
        print(laser.laser_on())
        data: dict = laser.get_single_scan()
        print(laser.reset())
        fixed_data = {}
        for rev_ang in data:
            fixed_data[-float(rev_ang)] = data[rev_ang]

        front_obtacle_distances: list = [
            fixed_data[i] for i in fixed_data if abs(int(float(i))) <= front_angle
        ]
        print(front_obtacle_distances)
        front_obtacle_distance: int = sum(front_obtacle_distances) / len(front_obtacle_distances)

        if front_obtacle_distance < turn_while_obstacle_is_closed_than:
            print("I will change my direction, measured dist:", front_obtacle_distance)
        else:
            print("I will move forward, measured dist:", front_obtacle_distance)

        # print(dumps(fixed_data, indent=4))
        print(laser.laser_off())

        sleep(3)
