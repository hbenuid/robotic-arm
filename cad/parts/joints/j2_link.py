"""j2_link - import wrapper around the SolidWorks reference (reference/j2_link.step).

SolidWorks product: 'Joint 2 change 8126'
Source export:      step/Joint 2 change 8126.STEP
Reference: mm units, 1 solid(s), volume 285470.5 mm^3,
           bbox size (300, 90, 33.5) mm, bbox min (-255, -45, 0) mm.
In the arm: x1 (j2_link#1).

Not yet parametric: j2_link() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from build123d import Location
from cadgen import step
from lib import reference

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once j2_link() is parametric build123d
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def j2_link():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


if __name__ == "__main__":
    j2_link()   # build: writes the sibling j2_link.step (preview: ./cadtool show parts/joints/j2_link.py)
