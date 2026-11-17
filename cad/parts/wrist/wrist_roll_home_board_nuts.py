"""wrist_roll_home_board_nuts - purchased (COTS) part: the 2 M2 hex nuts (ISO 4032) on the wrist_roll home sensor's
KY-003, in hex pockets in its fin's back.

In the arm: one pattern (wrist_roll_home_board_nuts, a row of wrist_roll_drive#1 in the KY-003's frame -
lib/wrist/layout.py home_sensor_frame()): each in its pocket in the fin's back (wrist_roll_home_fin), on the pocket's
floor - the M2 from the board's side (wrist_roll_home_board_screws) pulls it there; the hex stops it turning.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut()) and
tests/wrist/test_roll_home.py locks its numbers. Frame: the KY-003's, the nuts' bearing faces on the pockets' floors
(z = nut_pocket_depth - the fin's thickness), the nuts toward -Z.
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import WRIST_ROLL_HOME_BOARD_NUTS_MASS_G
from lib.wrist import DEFAULT
from lib.wrist.layout import home_board_points, home_fin_t

HOME = DEFAULT.roll.home
POINTS = home_board_points(DEFAULT)
CORNER_DEG = 90.0   # a corner on +Y, as the pockets'
FLOOR = HOME.nut_pocket_depth - home_fin_t(DEFAULT)   # the pockets' floors in the KY-003's frame

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_HOME_BOARD_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_nut.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "dropped into the hex pockets in the wrist_roll home sensor fin's back, on the KY-003's M2s"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nuts: the geometry until a vendor model exists."""
    return pattern([hex_nut(HOME.board_nut, math.radians(CORNER_DEG), (x, y), FLOOR - HOME.board_nut.h) for x, y in POINTS])


@step(out="wrist_roll_home_board_nuts.step")
def wrist_roll_home_board_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_home_board_nuts()   # build: writes the sibling wrist_roll_home_board_nuts.step
