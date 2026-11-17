"""wrist_roll_home_magnet - purchased (COTS) part: the wrist_roll home magnet, an N52 NdFeB disc (lib/wrist/params.py
WristRollHomeParams magnet_dia / magnet_t: Ø6 x 2, the elbow's).

Glued on the wrist roll pulley face's rim (wrist_roll_pulley, printed), at the face's middle - it spans the face's 6 mm
-, its SOUTH pole out, toward the KY-003 on its fin (ky003_hall_sensor on wrist_roll_home_fin): the A3144 switches on a
south pole at its branded face. The rim's curve leaves 0.2 at the disc's edges for the glue. It turns with the pulley
(wrist_roll_link) and stands over the chip at wrist_roll WRIST_ROLL_HOME_DEG (lib/params.py): in the zero pose
lib/wrist/layout.py home_magnet_deg() round from the module's +X, between two of the connector's nut slots - 0.69 off
the nearer one's mouth at the rim: keep the glue out of it. Mark the pole before gluing (the module's LED lights with
the south pole toward the chip); nothing on the rim locates it, and 1 mm off its mark round the axis moves home 2.6 deg
(docs/open_issues.md).

In the arm: x1 (wrist_roll_home_magnet, a row of wrist_roll_drive#1 - the rotor, wrist_roll_link).

No catalog model (none adopted), no export and NO reference file: a part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (a plain disc) and tests/wrist/test_roll_home.py locks its numbers.
Frame: axis on Z, its south pole's face on z = 0, the disc in +Z (toward the rim it is glued to).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import WRIST_ROLL_HOME_MAGNET_MASS_G
from lib.wrist import DEFAULT

HOME = DEFAULT.roll.home

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_HOME_MAGNET_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{HOME.magnet_dia:g} x {HOME.magnet_t:g} mm N52 NdFeB disc magnet (magnetised through its thickness)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = ("the wrist_roll home magnet: glued on the roll pulley face's rim on its mark, south pole out (check it on "
                 "the KY-003: its LED lights)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain disc: the geometry until a vendor model exists."""
    return pattern([cylinder(HOME.magnet_dia / 2.0, HOME.magnet_t)])


@step(out="wrist_roll_home_magnet.step")
def wrist_roll_home_magnet():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_home_magnet()   # build: writes the sibling wrist_roll_home_magnet.step
