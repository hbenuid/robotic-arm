"""roll_home_magnet - purchased (COTS) part: the forearm_roll home magnet, an N52 NdFeB disc (lib/forearm/params.py
RollHomeParams magnet_dia / magnet_t: Ø6 x 3, base_yaw's).

Pressed flush into its pocket in the roll shaft's rear face (forearm_roll_shaft, printed), RollHomeParams chip_r off
the roll axis in line with the stop lug, its SOUTH pole out - toward the KY-003 on its carrier behind the shaft
(ky003_hall_sensor on roll_home_carrier): the A3144 switches on a south pole at its branded face. It turns with the
shaft (forearm_link) and stands over the chip at forearm_roll FOREARM_ROLL_HOME_DEG (lib/params.py). Check the pole
before pressing it in (the module's LED lights with the south pole toward the chip); the pocket locates it.

In the arm: x1 (roll_home_magnet, a row of forearm_roll_drive#1 - the rotor, forearm_link).

No catalog model (none adopted), no export and NO reference file: a part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (a plain disc) and tests/forearm/test_roll_home.py locks its numbers.
Frame: axis on Z, its south pole's face on z = 0, the disc in +Z (into the shaft).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.forearm import DEFAULT
from lib.geom import cylinder
from lib.params import ROLL_HOME_MAGNET_MASS_G

HOME = DEFAULT.drive.home

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_HOME_MAGNET_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{HOME.magnet_dia:g} x {HOME.magnet_t:g} mm N52 NdFeB disc magnet (magnetised through its thickness)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = ("the forearm_roll home magnet: pressed flush into its pocket in the roll shaft's rear face, south pole out "
                 "(check it on the KY-003: its LED lights)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain disc: the geometry until a vendor model exists."""
    return pattern([cylinder(HOME.magnet_dia / 2.0, HOME.magnet_t)])


@step(out="roll_home_magnet.step")
def roll_home_magnet():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_home_magnet()   # build: writes the sibling roll_home_magnet.step
