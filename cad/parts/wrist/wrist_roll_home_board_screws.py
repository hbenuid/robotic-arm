"""wrist_roll_home_board_screws - purchased (COTS) part: the KY-003's 2 M2 socket head cap screws (ISO 4762) on the
wrist_roll home sensor's fin.

In the arm: one pattern (wrist_roll_home_board_screws, a row of wrist_roll_drive#1 in the KY-003's frame -
lib/wrist/layout.py home_sensor_frame()): the heads on the board's component side at its two holes (ky003_hall_sensor,
KY003_HOLE_DIA), the shanks through the board and the fin into the nuts in the fin's back (wrist_roll_home_board_nuts),
the tips at the fin's back. Length: WristRollHomeParams.board_screw_len.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py shcs()) and
tests/wrist/test_roll_home.py locks its numbers. Frame: the KY-003's, the heads' bearing face on the board's component
side (z = KY003_BOARD_T), the heads in +Z, the shanks in -Z.
"""
import pathlib

import build123d as bd
from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import KY003_BOARD_T, WRIST_ROLL_HOME_BOARD_SCREWS_MASS_G
from lib.wrist import DEFAULT
from lib.wrist.layout import home_board_points

HOME = DEFAULT.roll.home
POINTS = home_board_points(DEFAULT)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_HOME_BOARD_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_screw.d:g} x {HOME.board_screw_len:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "the wrist_roll home sensor's KY-003 onto its fin, from the board's side into the M2 nuts in the fin's back"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screws, heads on the board: the geometry until a vendor model exists."""
    return pattern([bd.Pos(x, y, KY003_BOARD_T) * bd.Rot(180.0, 0.0, 0.0) * shcs(HOME.board_screw, HOME.board_screw_len)
                    for x, y in POINTS])


@step(out="wrist_roll_home_board_screws.step")
def wrist_roll_home_board_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_home_board_screws()   # build: writes the sibling wrist_roll_home_board_screws.step
