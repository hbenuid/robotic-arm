"""yaw_home_board_screws - purchased (COTS) part: the KY-003's 2 M2 socket head cap screws (ISO 4762) on the base_yaw
home bracket.

In the arm: one pattern (yaw_home_board_screws#1, a mount on base#1 in the bracket's frame - lib/mounts.py
YAW_HOME_MOUNTS): the heads on the board's top at its two holes (ky003_hall_sensor, KY003_HOLE_DIA), the shanks down
through the board, the bracket's seat pad and its plate (yaw_home_bracket) into the nuts under it
(yaw_home_board_nuts). The board goes on the bracket before the bracket goes up. Length: YawHomeParams.board_screw_len.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py shcs(), turned over) and
tests/base/test_yaw_home.py locks its numbers. Frame: the bracket's (lib/base/layout.py home_bracket_frame()), the heads'
bearing face on the board's top (z = bracket_t + pad_h + KY003_BOARD_T), the shanks in -Z.
"""
import pathlib

import build123d as bd
from cadgen import step

from lib.base import DEFAULT, home_board_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import KY003_BOARD_T, YAW_HOME_BOARD_SCREWS_MASS_G

HOME = DEFAULT.home
POINTS = home_board_points(DEFAULT)
TOP = HOME.bracket_t + HOME.pad_h + KY003_BOARD_T   # the board's top in the bracket's frame

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_HOME_BOARD_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_screw.d:g} x {HOME.board_screw_len:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "the base_yaw home sensor's KY-003 onto its bracket, through the board's own holes into M2 nuts"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screws, heads up on the board: the geometry until a vendor model exists."""
    return pattern([bd.Pos(x, -z, TOP) * bd.Rot(180.0, 0.0, 0.0) * shcs(HOME.board_screw, HOME.board_screw_len)
                    for x, z in POINTS])


@step(out="yaw_home_board_screws.step")
def yaw_home_board_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_home_board_screws()   # build: writes the sibling yaw_home_board_screws.step
