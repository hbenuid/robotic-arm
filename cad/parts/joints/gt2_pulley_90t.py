"""gt2_pulley_90t - import wrapper around the SolidWorks reference (reference/gt2_pulley_90t.step).

SolidWorks product: 'GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric'
Source export:      step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP
Reference: mm units, 1 solid(s), volume 24810.7 mm^3,
           bbox size (59.188, 21.4, 59.188) mm, bbox min (-29.594, -13.2, -29.594) mm.
In the arm: x2 (gt2_pulley_90t#3 at the elbow, #4 at the wrist - mounts that re-seat the SolidWorks #1 / #2, retired,
PULLEY_SEAT_SHIFT out along the axis: its Ø30 x 7 journal in the lower 6806 of the joint, its Ø34.76 ring under that
bearing's inner ring - lib/mounts.py).

Printed 90-tooth GT2 pulley (config 'GT2 Pulley - Parametric'); 842 faces, ~1 s to import. Used at J2 and J3.

Not yet parametric: gt2_pulley_90t() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from cadgen import step

from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gt2_pulley_90t() is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gt2_pulley_90t():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gt2_pulley_90t()   # build: writes the sibling gt2_pulley_90t.step
