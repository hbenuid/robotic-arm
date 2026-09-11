"""j1_link - import wrapper around the SolidWorks reference (reference/j1_link.step).

SolidWorks product: 'first joint edit 62126'
Source export:      step/first joint edit 62126.STEP
Reference: mm units, 1 solid(s), volume 354048.8 mm^3,
           bbox size (300, 34, 90) mm, bbox min (-45, -32.5, -45) mm.
In the arm: x1 (j1_link#1).

Not yet parametric: j1_link() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from build123d import Location
from cadgen import step
from lib import reference

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once j1_link() is parametric build123d
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def j1_link():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


if __name__ == "__main__":
    j1_link()   # build: writes the sibling j1_link.step (preview: ./cadtool show parts/base/j1_link.py)
