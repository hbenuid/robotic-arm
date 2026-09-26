"""gt2_pulley_90t - the SolidWorks reference (reference/gt2_pulley_90t.step) with its 4 bolt holes opened to M4 clearance.

SolidWorks product: 'GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric'
Source export:      step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP
Reference: mm units, 1 solid(s), volume 24810.7 mm^3,
           bbox size (59.188, 21.4, 59.188) mm, bbox min (-29.594, -13.2, -29.594) mm.
In the arm: x2 (gt2_pulley_90t#3 at the elbow, #4 at the wrist - mounts that re-seat the SolidWorks #1 / #2, retired,
PULLEY_SEAT_SHIFT out along the axis: its Ø30 x 7 journal in the lower 6806 of the joint, its Ø34.76 ring under that
bearing's inner ring; the elbow's also turned 45 deg about its axis with the elbow block's bolt pattern - lib/mounts.py).
Its 4 bolt holes (lib/belts.py pulley_90t_bolt_points(), on its X / Z axes) run through the hub end to end
(GT2_PULLEY_90T_FACE_Y): the pulley bolts' heads sit on its outer face (elbow_pulley_screws, wrist_pulley_screws).

Printed 90-tooth GT2 pulley (config 'GT2 Pulley - Parametric'); 842 faces, ~1 s to import. Used at J2 and J3.

Diverged, not parametric: the model is the reference geometry in the SolidWorks part-file frame with the holes
opened from the export's Ø3.9 (under an M4's shank) to M4_CLEAR - REFERENCE_BUILD is the untouched reference
(tests/test_reference_match.py), tests/test_mounts.py locks the holes. See parts/_templates/wrapper.py for how to
convert it to build123d.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import step

from lib import reference
from lib.datum import IDENTITY, to_location
from lib.geom import single_solid, through
from lib.params import GT2_PULLEY_90T_FACE_Y, M4_CLEAR, pulley_90t_bolt_points

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = True              # its own build (the reference with the holes opened), not the export
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)


def REFERENCE_BUILD():
    """The untouched SolidWorks geometry (the reference-match lock)."""
    return reference.load(REFERENCE)


def _bolt_holes():
    """The 4 M4 clearance holes along the pulley's +Y, through the hub end to end."""
    y0, y1 = GT2_PULLEY_90T_FACE_Y
    return [bd.Rot(-90.0, 0.0, 0.0) * through(M4_CLEAR / 2.0, y1 - y0, (x, -z), z0=y0) for x, z in pulley_90t_bolt_points()]


@step
def gt2_pulley_90t():
    """Return the reference geometry with the holes opened, as a labelled Solid in this part's local frame."""
    shape = reference.load(REFERENCE)
    for hole in _bolt_holes():
        shape = shape - hole
    shape = single_solid(shape).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gt2_pulley_90t()   # build: writes the sibling gt2_pulley_90t.step
