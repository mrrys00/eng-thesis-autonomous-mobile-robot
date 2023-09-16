"""
Functions to control robot and it's parameters based on MiabotProUserManual_1_3.pdf
"""

from serial import Serial

import commands_list as cmds


class Commands():
    def __init__(self, serial_) -> None:
        self.serial: Serial = serial_
        self.last_get_message: str or list = None

    def send_command(self, cmd: str) -> str or None:
        """returns info about send data success and message"""
        self.serial.write(str.encode(cmd))
        self.last_get_message = self.serial.readall().decode()

        return self.last_get_message

    def stop(self) -> str or None:
        """stops robot immediately"""
        return self.send_command(cmd=cmds.command_stop)

    def version_info(self) -> str or None:
        """returns version info"""
        return self.send_command(cmd=cmds.command_version)

    def set_speed_decimal(self, left_wheel: int, right_wheel: int) -> str or None:
        """set decimal speed"""
        command = cmds.command_set_decimal_speed(
            left_=left_wheel,
            right_=right_wheel
        )
        return self.send_command(cmd=command)

    # def set_speed_byte(self, left_wheel: int, right_wheel: int) -> str or None:
    #     """set byte speed"""
    #     pass

    def set_step_distance(self, direction_: int, distance_: int) -> str or None:
        """set step distance"""
        command = cmds.command_set_step_distance(
            direction_=direction_,
            distance_=distance_
        )
        return self.send_command(cmd=command)

    def turn_left(self) -> str or None:
        """turn left"""
        return self.send_command(cmd=cmds.command_turn_left)

    def turn_right(self) -> str or None:
        """turn right"""
        return self.send_command(cmd=cmds.command_turn_right)

    def step_forward(self) -> str or None:
        """step forward"""
        return self.send_command(cmd=cmds.command_step_forward)

    def step_backward(self) -> str or None:
        """step backward"""
        return self.send_command(cmd=cmds.command_step_backward)
