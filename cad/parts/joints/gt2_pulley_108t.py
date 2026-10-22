"""gt2_pulley_108t - the printed 108T GT2 pulley of the elbow belt (parametric build123d, lib/pulley/body.py
build_pulley(ELBOW), in the 90T's part frame - origin on the axis at the lower flange's top, +Y up the axis toward the
outer face).

The 90T (parts/joints/gt2_pulley_90t) with GT2_PULLEY_108T_TEETH: the same hub - its Ø30 journal in the lower elbow
bearing, its ring under that bearing's inner ring -, the same web height, bore and 4x M4 through the hub end to end,
counterbored for the heads (the elbow pulley bolts: elbow_pulley_screws, elbow_pulley_nuts); the toothed rim grown with
the teeth, the 90T's wall kept under them. With the elbow motor's 20T it gives the elbow ratio (lib/params.py
ELBOW_RATIO). Every number: lib/pulley/params.py (ELBOW).

In the arm: x1 (gt2_pulley_108t#1 at the elbow, a mount that re-seats the SolidWorks gt2_pulley_90t#1, retired, on the
lower elbow bearing - lib/mounts.py).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/pulley/test_gt2_pulley_108t.py.
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.pulley import ELBOW
from lib.pulley.body import build_pulley

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/pulley/test_gt2_pulley_108t.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the 90T's part frame


@step
def gt2_pulley_108t():
    """The pulley at its local origin (the lower flange's top on the axis); the assembly owns placement."""
    part = build_pulley(ELBOW)
    part.label = NAME
    return part


if __name__ == "__main__":
    gt2_pulley_108t()   # build: writes the sibling gt2_pulley_108t.step
