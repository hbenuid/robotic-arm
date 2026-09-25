"""j1_cap - import wrapper around the SolidWorks reference (reference/j1_cap.step).

SolidWorks product: 'first joint cap 8726'
Source export:      step/first joint cap 8726.STEP
Reference: inch units, 1 solid(s), volume 150356.6 mm^3,
           bbox size (300, 90, 27.228) mm, bbox min (-838.697, 855.589, -27.228) mm.
In the arm: x1 (j1_cap#1).

Inch-unit SolidWorks export (OCCT converts to mm). Geometry sits far from the part origin; placements.json compensates - pick a sane origin via LOCAL_FROM_REF when converting.

Not yet parametric: j1_cap() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step

from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once j1_cap() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def j1_cap():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    j1_cap()   # build: writes the sibling j1_cap.step
