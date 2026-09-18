"""gripper_end - import wrapper around the SolidWorks reference (reference/gripper_end.step).

SolidWorks product: 'Gripper End_Gripper End'
Source export:      step/Gripper End_Gripper End.STEP
Reference: mm units, 1 solid(s), volume 11497.7 mm^3,
           bbox size (27.123, 30, 25.123) mm, bbox min (-13.561, 0, -12.561) mm.
In the arm: x2 (gripper_end#1, gripper_end#2).

Not yet parametric: gripper_end() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_end() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_end():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_end()   # build: writes the sibling gripper_end.step (preview: ./cadtool show parts/gripper/gripper_end.py)
