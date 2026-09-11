"""base - import wrapper around the SolidWorks reference (reference/base.step).

SolidWorks product: 'base of robot arm 62126'
Source export:      step/base of robot arm 62126.STEP
Reference: mm units, 1 solid(s), volume 286169.1 mm^3,
           bbox size (168.178, 95.807, 106.679) mm, bbox min (-53.339, -100.9, -53.339) mm.
In the arm: x1 (base#1).

Not yet parametric: base() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from build123d import Location
from cadgen import step
from lib import reference

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once base() is parametric build123d
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def base():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


if __name__ == "__main__":
    base()   # build: writes the sibling base.step (preview: ./cadtool show parts/base/base.py)
