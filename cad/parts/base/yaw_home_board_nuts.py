"""yaw_home_board_nuts - purchased (COTS) part: the KY-003's 2 M2 hex nuts (ISO 4032) under the base_yaw home bracket.

In the arm: one pattern (yaw_home_board_nuts#1, a mount on base#1 in the bracket's frame - lib/mounts.py
YAW_HOME_MOUNTS): their bearing faces on the bracket's underside (yaw_home_bracket) on the board's 2 M2s
(yaw_home_board_screws).

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut(): no thread, the bore at the
nominal diameter) and tests/base/test_yaw_home.py locks its numbers. Frame: the bracket's (lib/base/layout.py
home_bracket_frame()), the nuts under z = 0, a corner on +Y.
"""
import math
import pathlib

from cadgen import step

from lib.base import DEFAULT, home_board_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import YAW_HOME_BOARD_NUTS_MASS_G

HOME = DEFAULT.home
POINTS = home_board_points(DEFAULT)
CORNER_DEG = 90.0   # a corner on +Y: the flats face +/- X

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_HOME_BOARD_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_nut.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "under the base_yaw home bracket, on the KY-003's M2s"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nuts: the geometry until a vendor model exists."""
    nut = HOME.board_nut
    return pattern([hex_nut(nut, math.radians(CORNER_DEG), (x, -z), -nut.h) for x, z in POINTS])


@step(out="yaw_home_board_nuts.step")
def yaw_home_board_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_home_board_nuts()   # build: writes the sibling yaw_home_board_nuts.step
