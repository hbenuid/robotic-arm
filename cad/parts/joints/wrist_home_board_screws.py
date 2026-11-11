"""wrist_home_board_screws - purchased (COTS) part: the KY-003's 2 M2 socket head cap screws (ISO 4762) on the
wrist_pitch home sensor's clip.

In the arm: one pattern (wrist_home_board_screws#1, a mount on j2_link#1 in the KY-003's frame - lib/mounts.py
WRIST_HOME_MOUNTS): the heads on the clip's plate's underside at the board's two holes (ky003_hall_sensor,
KY003_HOLE_DIA), the shanks up through the plate's teardrop holes and the board into the nuts on its component side
(wrist_home_board_nuts). Length: WristHomeParams.board_screw_len.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py shcs()) and
tests/forearm/test_wrist_home.py locks its numbers. Frame: the KY-003's (lib/forearm/layout.py home_sensor_frame()),
the heads' bearing face on the plate's underside (z = -plate_t), the shanks in +Z.
"""
import pathlib

import build123d as bd
from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import DEFAULT
from lib.forearm.layout import home_board_points
from lib.params import WRIST_HOME_BOARD_SCREWS_MASS_G

HOME = DEFAULT.home
POINTS = home_board_points(DEFAULT)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_HOME_BOARD_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_screw.d:g} x {HOME.board_screw_len:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "the wrist_pitch home sensor's KY-003 onto its clip, through the board's own holes into M2 nuts"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screws, heads under the plate: the geometry until a vendor model exists."""
    return pattern([bd.Pos(x, y, -HOME.plate_t) * shcs(HOME.board_screw, HOME.board_screw_len) for x, y in POINTS])


@step(out="wrist_home_board_screws.step")
def wrist_home_board_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_home_board_screws()   # build: writes the sibling wrist_home_board_screws.step
