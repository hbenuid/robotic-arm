"""gripper_link_1 - import wrapper around the SolidWorks reference (reference/gripper_link_1.step).

SolidWorks product: 'Gripper link 1_Gripper link 1'
Source export:      step/Gripper link 1_Gripper link 1.STEP
Reference: mm units, 1 solid(s), volume 766.6 mm^3,
           bbox size (34.2, 3.5, 7.2) mm, bbox min (-30.6, 0, -3.6) mm.
In the arm: x2 (gripper_link_1#1, gripper_link_1#2).

Not yet parametric: gripper_link_1() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_link_1() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_link_1():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_link_1()   # build: writes the sibling gripper_link_1.step (preview: ./cadtool show parts/gripper/gripper_link_1.py)
