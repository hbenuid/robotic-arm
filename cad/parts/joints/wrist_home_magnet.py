"""wrist_home_magnet - purchased (COTS) part: the wrist_pitch home magnet, an N52 NdFeB disc (lib/forearm/params.py
WristHomeParams magnet_dia / magnet_t: Ø6 x 2, the elbow's).

Glued onto the wrist 90T's outer face (gt2_pulley_90t, printed), WristHomeParams chip_x / chip_y off the wrist axis, its
SOUTH pole out - toward the KY-003 on its clip (ky003_hall_sensor on wrist_home_clip): the A3144 switches on a south pole
at its branded face. It turns with the 90T (wrist_pitch_link) and stands over the chip at wrist_pitch WRIST_PITCH_HOME_DEG
(lib/params.py). Mark the pole before gluing (the module's LED lights with the south pole toward the chip); nothing on the
face locates it, and 1 mm off its mark round the axis moves home 2.4 deg (docs/open_issues.md).

In the arm: x1 (wrist_home_magnet#1, a mount on gt2_pulley_90t#4 - lib/mounts.py WRIST_HOME_MOUNTS; wrist_pitch_link).

No catalog model (none adopted), no export and NO reference file: a part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (a plain disc) and tests/forearm/test_wrist_home.py locks its numbers.
Frame: axis on Z, its south pole's face on z = 0, the disc in +Z (toward the face it is glued to).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.forearm import DEFAULT
from lib.geom import cylinder
from lib.params import WRIST_HOME_MAGNET_MASS_G

HOME = DEFAULT.home

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_HOME_MAGNET_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{HOME.magnet_dia:g} x {HOME.magnet_t:g} mm N52 NdFeB disc magnet (magnetised through its thickness)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = ("the wrist_pitch home magnet: glued onto the wrist 90T's outer face on its mark, south pole out (check it "
                 "on the KY-003: its LED lights)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain disc: the geometry until a vendor model exists."""
    return pattern([cylinder(HOME.magnet_dia / 2.0, HOME.magnet_t)])


@step(out="wrist_home_magnet.step")
def wrist_home_magnet():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_home_magnet()   # build: writes the sibling wrist_home_magnet.step
