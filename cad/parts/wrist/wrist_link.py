"""wrist_link - import wrapper around the SolidWorks reference (reference/wrist_link.step).

SolidWorks product: 'final component arm qwrist movement'
Source export:      step/final component arm qwrist movement.STEP
Reference: mm units, 1 solid(s), volume 113601.4 mm^3,
           bbox size (124.446, 78, 44) mm, bbox min (-84.446, -39, -0) mm.
In the arm: x1 (wrist_link#1).

SolidWorks name 'final component arm qwrist movement' (wrist link).

Not yet parametric: wrist_link() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once wrist_link() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def wrist_link():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    wrist_link()   # build: writes the sibling wrist_link.step
