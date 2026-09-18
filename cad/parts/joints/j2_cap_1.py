"""j2_cap_1 - import wrapper around the SolidWorks reference (reference/j2_cap_1.step).

SolidWorks product: 'cap 1 joint 2 8726'
Source export:      step/cap 1 joint 2 8726.STEP
Reference: mm units, 1 solid(s), volume 97819.5 mm^3,
           bbox size (245.461, 90, 14.5) mm, bbox min (-200.461, -45, -14.5) mm.
In the arm: x1 (j2_cap_1#1).

Geometry is offset from the part origin; placements.json compensates.

Not yet parametric: j2_cap_1() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once j2_cap_1() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def j2_cap_1():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    j2_cap_1()   # build: writes the sibling j2_cap_1.step (preview: ./cadtool show parts/joints/j2_cap_1.py)
