"""Project configuration: CAN bus + joint table."""

# CAN bus
CAN_CHANNEL = "/dev/tty.usbmodem1101"  # adjust to actual CANable path
CAN_BITRATE = 500000                    # MKS factory default

# Motion defaults
DEFAULT_SPEED = 300   # RPM, safe for benchtop testing
DEFAULT_ACC = 2       # 0-255, low = gentle ramp

# Motor / encoder
ENCODER_COUNTS_PER_REV = 0x4000  # 16384, MKS SERVO42D internal encoder

# Joint table: (name, can_id, gear_ratio)
# can_id matches the value set on each motor's on-board menu.
# gear_ratio = output revolutions per motor revolution; use 1.0 for direct drive.
JOINTS = [
    ("J1", 0x01, 1.0),
    ("J2", 0x02, 1.0),
    ("J3", 0x03, 1.0),
]
