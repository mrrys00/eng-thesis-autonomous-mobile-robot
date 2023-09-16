"""
TO DO - docs, mre commands
"""

command_stop = "[s]"
command_version = "[?]"

command_turn_left = "[<]"
command_turn_right = "[>]"
command_step_forward = "[^]"
command_step_backward = "[v]"


def command_set_decimal_speed(left_: int, right_: int) -> str or None:
    """returns command to set decimal speed"""
    if type(left_) is type(int) and \
            type(right_) is type(int) and \
            left_ > 0 and right_ > 0 and \
            left_ < 100 and right_ < 100:
        left_, right_ = str(left_), str(right_)
        if len(left_) == 1:
            left_ = "0" + left_
        if len(right_) == 1:
            right_ = "0" + right_

        return f"[=<{left_}l>,<{right_}r>]"

    return None


def command_set_step_distance(direction_: str, distance_: int) -> str or None:
    """returns command to set decimal speed"""
    if direction_ in ["<", ">", "^", "v"] and \
            distance_ >= 0 and distance_ < 2 ^ 31:
        return f"[=<{direction_}>,<{distance_}>]"

    return None
