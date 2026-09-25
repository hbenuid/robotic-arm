"""gripper_cover - import wrapper around the SolidWorks reference (reference/gripper_cover.step).

SolidWorks product: 'Gripper Cover_Gripper Cover'
Source export:      step/Gripper Cover_Gripper Cover.STEP
Reference: mm units, 1 solid(s), volume 13140.6 mm^3,
           bbox size (44, 10.5, 70) mm, bbox min (-22, 0, -35) mm.
In the arm: x1 (gripper_cover#1).

Not yet parametric: gripper_cover() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_cover() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_cover():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_cover()   # build: writes the sibling gripper_cover.step
