"""gripper_clamp_bracket - import wrapper around the SolidWorks reference (reference/gripper_clamp_bracket.step).

SolidWorks product: 'brack for hand cmap'
Source export:      step/brack for hand cmap.STEP
Reference: mm units, 1 solid(s), volume 11175.6 mm^3,
           bbox size (26.2, 64, 43.2) mm, bbox min (-66.2, -32.001, -43.2) mm.
In the arm: x1 (gripper_clamp_bracket#1).

SolidWorks name 'brack for hand cmap' (bracket for the hand clamp).

Not yet parametric: gripper_clamp_bracket() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_clamp_bracket() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_clamp_bracket():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_clamp_bracket()   # build: writes the sibling gripper_clamp_bracket.step (preview: ./cadtool show parts/wrist/gripper_clamp_bracket.py)
