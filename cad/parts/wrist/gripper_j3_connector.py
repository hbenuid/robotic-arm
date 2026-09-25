"""gripper_j3_connector - import wrapper around the SolidWorks reference (reference/gripper_j3_connector.step).

SolidWorks product: 'Gripper to J3 connector 7726_Gripper to J3 connector'
Source export:      step/Gripper to J3 connector 7726_Gripper to J3 connector.STEP
Reference: mm units, 2 solid(s), volume 4713.0 mm^3,
           bbox size (23, 12.5, 46) mm, bbox min (-11.5, 0, -23) mm.
In the arm: x1 (gripper_j3_connector#1).

Two solid bodies in one part (kept as a flat Compound; registered in tests MULTI_BODY).

Not yet parametric: gripper_j3_connector() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_j3_connector() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_j3_connector():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_j3_connector()   # build: writes the sibling gripper_j3_connector.step
