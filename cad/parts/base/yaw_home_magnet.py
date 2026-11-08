"""yaw_home_magnet - purchased (COTS) part: the base_yaw home magnet, an N52 NdFeB disc (lib/sensors.py HOME_MAGNET_*).

Glued into its pocket in the 120T's outer face (gt2_pulley_120t, lib/pulley/params.py YAW magnet), its SOUTH pole out,
flush with the face - which faces down, toward the KY-003 under it (ky003_hall_sensor on yaw_home_bracket): the A3144
switches on a south pole at its branded face. It turns with the 120T and stands over the chip at base_yaw
BASE_YAW_HOME_DEG (lib/params.py). Mark the pole before gluing: the module's LED lights with the south pole toward the chip.

In the arm: x1 (yaw_home_magnet#1, a mount on gt2_pulley_120t#1 - lib/mounts.py YAW_HOME_MOUNTS; shoulder_link).

No catalog model (none adopted), no export and NO reference file: a part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (a plain disc) and tests/base/test_yaw_home.py locks its numbers. Frame:
axis on Z, its south pole's face on z = 0, the disc in +Z.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import HOME_MAGNET_DIA, HOME_MAGNET_T, YAW_HOME_MAGNET_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_HOME_MAGNET_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{HOME_MAGNET_DIA:g} x {HOME_MAGNET_T:g} mm N52 NdFeB disc magnet (magnetised through its thickness)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = ("the base_yaw home magnet: glued into the pocket in the 120T's outer face, south pole out (check it on "
                 "the KY-003: its LED lights)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain disc: the geometry until a vendor model exists."""
    return pattern([cylinder(HOME_MAGNET_DIA / 2.0, HOME_MAGNET_T)])


@step(out="yaw_home_magnet.step")
def yaw_home_magnet():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_home_magnet()   # build: writes the sibling yaw_home_magnet.step
