"""j1_link - the upper arm: the link between the shoulder_pitch and elbow_pitch axes (parametric build123d,
lib/upper_arm/link.py build_link(cfg), in the SolidWorks part frame - origin on the shoulder axis, the elbow axis at
x = SlabParams.elbow_x, +Y = N, the cycloidal drive's hub on the y = +1.5 top face).

A stadium plate, 90 wide (y -11.77..1.5 at the shoulder half, stepping down to y -24 under the elbow half) with a
r 44.5 lip on top. Shoulder end: the Ø50 4x M4 bolts of the drive's output hub around a 42.5 square opening, over
the elbow motor's 48 square pad (face y -32.5, a window in each wall - the +X one is the elbow belt's exit - the
NEMA 17 holes in its floor; lib/mounts.py places nema17_40mm#2 on it). Elbow end: the Ø80 recess, the Ø42 bore, a
Ø37.64 lip and the Ø42.2 seat from below on the elbow axis (the elbow's 6806-2RS pair, one each side of the lip -
lib/mounts.py). Between them: two through slots, the stepped slot (purpose unknown) and a Ø22.2 seat from each side
(the elbow drive's second stage, not modelled). DEFAULT (what the part builds) is SHORTENING shorter at the elbow
end (the elbow-end features move with the axis), leaves out the through slots and LEGACY's 10 Ø5.15 x 2 sockets in
the underside (j1_cap's dowel seats - the cap was removed 2026-09-25), puts the 4 NEMA 17
holes on a 31 square about the shoulder axis (the SolidWorks ones are 0.38 off and uneven) and the hub holes on the
drive's bolts (3.36 degrees from the SolidWorks ones). Every number: lib/upper_arm/params.py (UpperArmConfig;
measured on the reference 2026-09-24).

SolidWorks product: 'first joint edit 62126'
Source export:      step/first joint edit 62126.STEP
Reference: mm units, 1 solid(s), volume 354048.8 mm^3,
           bbox size (300, 34, 90) mm, bbox min (-45, -32.5, -45) mm.
In the arm: x1 (j1_link#1).

Conversion: build_link(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py);
the model builds DEFAULT (tests/upper_arm/ lock what DEFAULT changes).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.upper_arm import DEFAULT, LEGACY
from lib.upper_arm.link import build_link

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame
REF_BBOX_TOL = 0.02


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_link(LEGACY)


@step
def j1_link():
    """The upper arm at its local origin (the shoulder axis); the assembly owns placement."""
    part = build_link(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j1_link()   # build: writes the sibling j1_link.step
