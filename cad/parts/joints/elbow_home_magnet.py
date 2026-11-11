"""elbow_home_magnet - purchased (COTS) part: the elbow_pitch home magnet, an N52 NdFeB disc (lib/upper_arm/params.py
ElbowHomeParams magnet_dia / magnet_t: Ø6 x 2, thinner than base_yaw's and the shoulder's).

Glued onto the elbow 90T's outer face (gt2_pulley_90t, printed), ElbowHomeParams chip_x / chip_z off the elbow axis, its
SOUTH pole out - toward the KY-003 on its clip (ky003_hall_sensor on elbow_home_clip): the A3144 switches on a south pole
at its branded face. It turns with the 90T (elbow_link) and stands over the chip at elbow_pitch ELBOW_PITCH_HOME_DEG
(lib/params.py). Mark the pole before gluing (the module's LED lights with the south pole toward the chip); nothing on the
face locates it, and 1 mm off its mark round the axis moves home 2.4 deg (docs/open_issues.md).

In the arm: x1 (elbow_home_magnet#1, a mount on gt2_pulley_90t#3 - lib/mounts.py ELBOW_HOME_MOUNTS; elbow_link).

No catalog model (none adopted), no export and NO reference file: a part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (a plain disc) and tests/upper_arm/test_elbow_home.py locks its numbers.
Frame: axis on Z, its south pole's face on z = 0, the disc in +Z (toward the face it is glued to).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import ELBOW_HOME_MAGNET_MASS_G
from lib.upper_arm import DEFAULT

HOME = DEFAULT.home

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ELBOW_HOME_MAGNET_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{HOME.magnet_dia:g} x {HOME.magnet_t:g} mm N52 NdFeB disc magnet (magnetised through its thickness)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = ("the elbow_pitch home magnet: glued onto the elbow 90T's outer face on its mark, south pole out (check it "
                 "on the KY-003: its LED lights)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain disc: the geometry until a vendor model exists."""
    return pattern([cylinder(HOME.magnet_dia / 2.0, HOME.magnet_t)])


@step(out="elbow_home_magnet.step")
def elbow_home_magnet():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_home_magnet()   # build: writes the sibling elbow_home_magnet.step
