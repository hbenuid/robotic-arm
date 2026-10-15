"""gripper_j3_connector - the gripper module's link to the wrist roll's output (parametric build123d,
lib/wrist/connector.py build_connector(cfg), ConnectorParams), in the SolidWorks part frame: a plate in the x-z plane,
y 0..4.5 - a Ø46 disc cut to its +/- 11.5 flats -, the servo holder's and the gripper cover's 4 holes through it, its
+Y face toward the wrist.

DEFAULT (what the part builds): the plate and a hub bolted to wrist_roll_pulley's face - a Ø28 core, then a Ø42 flange
with 4x M3 on Ø36 (their heads between the flange and the plate, a key reaching each past the plate's flats) into nuts
in the pulley's face, a notch in the flange's rim over each pair of the plate's holes (their screws' key) - and the Ø10
cable bore (the servo's leads) through both: one body. LEGACY (REFERENCE_BUILD, tests/test_reference_match.py) is the
SolidWorks part: the plate and a Ø14 boss whose 20-tooth bore slid onto the pancake's 20T, two bodies - the bore as the
circle of its area. Every number: lib/wrist/params.py ConnectorParams (measured on the reference 2026-10-14).

SolidWorks product: 'Gripper to J3 connector 7726_Gripper to J3 connector'
Source export:      step/Gripper to J3 connector 7726_Gripper to J3 connector.STEP
Reference: mm units, 2 solid(s), volume 4713.0 mm^3,
           bbox size (23, 12.5, 46) mm, bbox min (-11.5, 0, -23) mm.
In the arm: x1 (gripper_j3_connector#1, in the gripper module - wrist_roll_link).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.wrist import DEFAULT, LEGACY
from lib.wrist.connector import build_connector

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_connector(LEGACY)


@step
def gripper_j3_connector():
    """The connector at its local origin (the plate's servo-side face, on its axis); the gripper module owns placement."""
    part = build_connector(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_j3_connector()   # build: writes the sibling gripper_j3_connector.step
