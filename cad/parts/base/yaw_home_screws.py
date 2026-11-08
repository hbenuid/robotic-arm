"""yaw_home_screws - purchased (COTS) part: the base_yaw home bracket's 2 M3 socket head cap screws (ISO 4762).

In the arm: one pattern (yaw_home_screws#1, a mount on base#1 in the bracket's frame - lib/mounts.py YAW_HOME_MOUNTS):
the heads under the bracket's root (yaw_home_bracket), the shanks up through it into the heat-set inserts in the motor
plate's underside (yaw_home_inserts). Turned from under the base. Length: YawHomeParams.screw_len.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py shcs(): no thread, the head's hex
socket) and tests/base/test_yaw_home.py locks its numbers. Frame: the bracket's (lib/base/layout.py
home_bracket_frame(): z = 0 on its underside, the heads' bearing face), the base's (x, z) at (x, -y).
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT, home_screw_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import YAW_HOME_SCREWS_MASS_G

HOME = DEFAULT.home
POINTS = home_screw_points(DEFAULT)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_HOME_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.screw.d:g} x {HOME.screw_len:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "the base_yaw home bracket up into its 2 inserts in the base's motor plate, from under the base"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screws: the geometry until a vendor model exists."""
    return pattern([shcs(HOME.screw, HOME.screw_len, (x, -z)) for x, z in POINTS])


@step(out="yaw_home_screws.step")
def yaw_home_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_home_screws()   # build: writes the sibling yaw_home_screws.step
