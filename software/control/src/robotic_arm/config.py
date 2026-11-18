"""This arm's hardware constants: the CAN bus, the MKS boards and the joint table.

The joint table must agree with the CAD (cad/robot/AGENTS.md "The reductions the software must match" and "The home
angles the software must match"): the joints are cad/robot/arm.urdf's, the reductions, limits and home angles
cad/lib/params.py's. cad/tests/test_robot.py loads this file by its path and checks every number against the CAD, so
it imports nothing outside the standard library.

What is not confirmed yet (cad/docs/open_issues.md): which board carries which CAN id, each joint's `sign`, and the
MKS firmware the boards run.
"""

from __future__ import annotations

from dataclasses import dataclass

# --- CAN bus --------------------------------------------------------------------------------------------------------
# The CLI's defaults; `uv run launcher` passes the serial port it finds (or your --channel). The package itself takes
# the channel and the python-can interface as parameters: "slcan" (a CANable with slcan firmware, a serial port) or
# "socketcan" (candleLight firmware on Linux, the bitrate set by `ip link`, not here).
CAN_INTERFACE = "slcan"
CAN_CHANNEL = "/dev/tty.usbmodem1101"   # a placeholder for direct runs
CAN_BITRATE = 500_000                   # the MKS factory default (CanRate 500K)

# --- MKS boards -----------------------------------------------------------------------------------------------------
ENCODER_COUNTS_PER_REV = 0x4000   # 16384 counts per motor turn (the 0x31 coordinate)
REPLY_TIMEOUT_S = 1.0             # how long a blocking request waits for its reply

# --- CLI motion defaults ----------------------------------------------------------------------------------------------
# Motor RPM: through this arm's reductions 60 RPM is 60 deg/s at base_yaw (6:1) and 180 deg/s at wrist_roll (2:1).
DEFAULT_SPEED = 60
DEFAULT_ACC = 2       # 0-255, low = a gentle ramp


@dataclass(frozen=True)
class JointConfig:
    """One driven joint of the arm.

    gear_ratio: output (joint) revolutions per motor revolution - the reciprocal of the CAD's reduction
    (cad/lib/params.py, motor revolutions per output revolution).
    sign: +1 when the board's encoder counts up as the joint turns positive (the URDF's direction), -1 when it counts
    down. Unconfirmed: +1 until measured on the arm.
    limits_deg: the arm's own hard limits (the URDF's). A caller narrows them per run (joint.JointLimits); an
    experiment's numbers never go here.
    home_deg: the joint angle at its home switch's edge - where the board's homing (0x91) puts its encoder zero.
    can_id: the board's CanID; None = no board on the bus for this joint.
    """

    name: str
    can_id: int | None
    gear_ratio: float
    sign: int
    limits_deg: tuple[float, float]
    home_deg: float


# Base to tip, in the URDF's chain order. CAN ids 1-6 in that order are what each board's CanID menu should be set to;
# that the boards carry them is unconfirmed.
JOINTS: tuple[JointConfig, ...] = (
    #           name              can_id  gear_ratio  sign  limits_deg         home_deg
    JointConfig("base_yaw",       0x01,   1 / 6,      +1,   (-175.0, 175.0),   -169.5),  # BASE_YAW_RATIO 120T/20T belt; the limit an [ESTIMATE]
    JointConfig("shoulder_pitch", 0x02,   1 / 21,     +1,   (-120.0, 67.0),    -118.0),  # CYCLOIDAL_RATIO
    JointConfig("elbow_pitch",    0x03,   1 / 22.5,   +1,   (-47.0, 144.0),    -41.0),   # ELBOW_RATIO 5:1 gearbox, then 90T/20T
    JointConfig("forearm_roll",   0x04,   1 / 4.5,    +1,   (-166.0, 166.0),   -160.0),  # FOREARM_ROLL_RATIO 90T/20T
    JointConfig("wrist_pitch",    0x05,   1 / 4.5,    +1,   (-150.0, 38.0),    -144.0),  # GT2_RATIO 90T/20T
    JointConfig("wrist_roll",     0x06,   1 / 2,      +1,   (-166.0, 166.0),   -165.0),  # WRIST_ROLL_RATIO
)
