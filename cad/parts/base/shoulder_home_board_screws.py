"""shoulder_home_board_screws - purchased (COTS) part: the KY-003's 2 M2 socket head cap screws (ISO 4762) on the
shoulder_pitch home sensor's tray.

In the arm: one pattern (shoulder_home_board_screws#1, a mount on j1_coupler#1 in the KY-003's frame - lib/mounts.py
SHOULDER_HOME_MOUNTS): the heads on the tray's plate's underside at the board's two holes (ky003_hall_sensor,
KY003_HOLE_DIA), in their recesses in the block, the shanks up through the plate and the board into the nuts on its
component side (shoulder_home_board_nuts). The board goes on the tray before the tray goes into the pocket; the recesses
run on out through the tray's underside for the key. Length: HomeTrayParams.board_screw_len.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py shcs()) and
tests/base/test_shoulder_home.py locks its numbers. Frame: the KY-003's (lib/yaw_coupler/layout.py home_sensor_frame()),
the heads' bearing face on the plate's underside (z = -plate_t), the shanks in +Z.
"""
import pathlib

import build123d as bd
from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import SHOULDER_HOME_BOARD_SCREWS_MASS_G
from lib.yaw_coupler import DEFAULT
from lib.yaw_coupler.layout import home_board_points

HOME = DEFAULT.home
POINTS = home_board_points(DEFAULT)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = SHOULDER_HOME_BOARD_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_screw.d:g} x {HOME.board_screw_len:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "the shoulder_pitch home sensor's KY-003 onto its tray, through the board's own holes into M2 nuts"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screws, heads under the plate: the geometry until a vendor model exists."""
    return pattern([bd.Pos(x, y, -HOME.plate_t) * shcs(HOME.board_screw, HOME.board_screw_len) for x, y in POINTS])


@step(out="shoulder_home_board_screws.step")
def shoulder_home_board_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    shoulder_home_board_screws()   # build: writes the sibling shoulder_home_board_screws.step
