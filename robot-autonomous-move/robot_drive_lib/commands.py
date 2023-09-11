"""
Functions to control robot and it's parameters based on MiabotProUserManual_1_3.pdf
"""

from serial import Serial

# Simple commands

class Commands():
    def __init__(self, serial_) -> None:
        self.serial: Serial = serial_

    def send_command(self, cmd: str) -> bool:
        """returns info about send data success and message"""
        self.serial.write(str.encode(cmd))
        print(self.serial.readall().decode())


