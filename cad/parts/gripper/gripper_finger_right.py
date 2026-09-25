"""gripper_finger_right - import wrapper around the SolidWorks reference (reference/gripper_finger_right.step).

SolidWorks product: 'Gripper Hand Right_Gripper Hand Left'
Source export:      step/Gripper Hand Right_Gripper Hand Left.STEP
Reference: mm units, 1 solid(s), volume 9069.3 mm^3,
           bbox size (26, 4, 105) mm, bbox min (0, 0, -8) mm.
In the arm: x2 (gripper_finger_right#1, gripper_finger_right#2).

Mirror configuration of gripper_finger_left (the SolidWorks export is still named '..._Gripper Hand Left').

Not yet parametric: gripper_finger_right() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_finger_right() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_finger_right():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_finger_right()   # build: writes the sibling gripper_finger_right.step
