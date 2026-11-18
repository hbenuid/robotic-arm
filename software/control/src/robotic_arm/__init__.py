"""The robotic-arm's motor control: its MKS SERVO42D boards over CAN, in layers (docs/architecture.md).

can_interface (CAN frames) -> motor_driver (the MKS protocol, one board) -> joint (degrees <-> counts, limits) ->
arm (every board on the bus); config holds the arm's constants, fake simulates boards for tests. The CLIs:
motor_control (interactive), launcher (finds the adapter, runs it), command_sweep (one board's command check).
"""

from robotic_arm.arm import Arm
from robotic_arm.can_interface import CANInterface, Frame
from robotic_arm.joint import Joint, JointLimits, JointSample, LimitError
from robotic_arm.motor_driver import Checksum, Cmd, MotorDriver, Reply

__all__ = [
    "Arm",
    "CANInterface",
    "Checksum",
    "Cmd",
    "Frame",
    "Joint",
    "JointLimits",
    "JointSample",
    "LimitError",
    "MotorDriver",
    "Reply",
]
