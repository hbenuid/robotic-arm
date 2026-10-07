"""gt2_pulley_120t - the printed 120T GT2 pulley of the base_yaw belt (parametric build123d, lib/pulley/body.py
build_pulley(YAW), in the 90T's part frame - origin on the axis at the lower flange's top, +Y up the axis toward the
outer face).

The 90T (parts/joints/gt2_pulley_90t) with GT2_PULLEY_120T_TEETH: the same hub - its Ø30 journal in the lower base
bearing, its ring under that bearing's inner ring -, the same web height, bore and 4x M4 through the hub end to end,
counterbored for the heads (the base_yaw pulley bolts: yaw_pulley_screws, yaw_pulley_nuts); the toothed rim grown with the teeth, the 90T's wall
kept under them. With the 48 mm motor's 20T it gives the base_yaw ratio (lib/params.py BASE_YAW_RATIO). Every number:
lib/pulley/params.py (YAW).

In the arm: x1 (gt2_pulley_120t#1, a mount on j1_coupler#1, upside down under it - hub up in the lower base bearing,
turned onto the coupler's diagonal holes - lib/mounts.py).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/pulley/test_gt2_pulley_120t.py.
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.pulley import YAW
from lib.pulley.body import build_pulley

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/pulley/test_gt2_pulley_120t.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the 90T's part frame


@step
def gt2_pulley_120t():
    """The pulley at its local origin (the lower flange's top on the axis); the assembly owns placement."""
    part = build_pulley(YAW)
    part.label = NAME
    return part


if __name__ == "__main__":
    gt2_pulley_120t()   # build: writes the sibling gt2_pulley_120t.step
