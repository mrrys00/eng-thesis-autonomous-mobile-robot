
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS

from ..configurations import config

def scanner_connect() -> Serial:
    """Returns scanner connection object"""
    return Serial(
        port=config.defaultScannerPort,
        baudrate=config.defaultScannerBaud,
        timeout=0.5
    )

def miabot_connect() -> Serial:
    """Returns Miabot connection object"""
    return Serial(
        port=config.defaultMiabotPort,
        baudrate=config.defaultMiabotBaud,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        bytesize=EIGHTBITS,
        timeout=1
    )

def serial_disconnect(serial_: Serial) -> bool:
    """Returns state of serial disconnection"""
    pass
