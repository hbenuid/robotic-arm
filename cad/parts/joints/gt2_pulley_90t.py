"""gt2_pulley_90t - the printed 90T GT2 pulley of the elbow and the wrist belts (parametric build123d,
lib/pulley/body.py build_pulley(cfg), in the SolidWorks part frame - origin on the axis at the lower flange's top, +Y up
the axis toward the outer face).

The hub's Ø30 journal fills the lower 6806 of its joint, a ring under that bearing's inner ring; the web carries the
toothed rim - the tooth band (the GT2 groove of lib/belts.py, lib/pulley/teeth.py) between two chamfered flanges -, open
underneath between the hub's step and the rim; a Ø12.5 bore; 4x M4 on the pulley's own X / Z axes (lib/belts.py
pulley_90t_bolt_points()) through the hub end to end (GT2_PULLEY_90T_FACE_Y): the pulley bolts' heads sit on its outer
face (elbow_pulley_screws, wrist_pulley_screws). Every number: lib/pulley/params.py (PulleyParams; measured on the
reference 2026-09-27). The base_yaw belt's pulley is this one with 120 teeth: parts/base/gt2_pulley_120t.

SolidWorks product: 'GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric'
Source export:      step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP
Reference: mm units, 1 solid(s), volume 24810.7 mm^3,
           bbox size (59.188, 21.4, 59.188) mm, bbox min (-29.594, -13.2, -29.594) mm.
In the arm: x2 (gt2_pulley_90t#3 at the elbow, #4 at the wrist - mounts that re-seat the SolidWorks #1 / #2, retired,
PULLEY_SEAT_SHIFT out along the axis: its journal in the lower 6806 of the joint, its ring under that bearing's inner
ring; the elbow's also turned about its axis with the elbow block's bolt pattern - lib/mounts.py).

Conversion: build_pulley(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py); the model
builds DEFAULT, its bolt holes opened from the export's Ø3.9 (under an M4's shank) to M4_CLEAR (tests/pulley/ lock what
DEFAULT changes, tests/test_mounts.py the holes in place).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.pulley import DEFAULT, LEGACY
from lib.pulley.body import build_pulley

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_pulley(LEGACY)


@step
def gt2_pulley_90t():
    """The pulley at its local origin (the lower flange's top on the axis); the assembly owns placement."""
    part = build_pulley(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    gt2_pulley_90t()   # build: writes the sibling gt2_pulley_90t.step
