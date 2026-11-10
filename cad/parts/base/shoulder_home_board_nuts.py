"""shoulder_home_board_nuts - purchased (COTS) part: the KY-003's 2 M2 hex nuts (ISO 4032) on the shoulder_pitch home
sensor's board.

In the arm: one pattern (shoulder_home_board_nuts#1, a mount on j1_coupler#1 in the KY-003's frame - lib/mounts.py
SHOULDER_HOME_MOUNTS): their bearing faces on the board's component side (ky003_hall_sensor), on its 2 M2s
(shoulder_home_board_screws) - under the turning shell's pillars, which pass them 4.5 mm or more off.

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut(): no thread, the bore at the
nominal diameter) and tests/base/test_shoulder_home.py locks its numbers. Frame: the KY-003's
(lib/yaw_coupler/layout.py home_sensor_frame()), the nuts on z = KY003_BOARD_T, a corner on +Y.
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import KY003_BOARD_T, SHOULDER_HOME_BOARD_NUTS_MASS_G
from lib.yaw_coupler import DEFAULT
from lib.yaw_coupler.layout import home_board_points

HOME = DEFAULT.home
POINTS = home_board_points(DEFAULT)
CORNER_DEG = 90.0   # a corner on +Y: the flats face +/- X

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = SHOULDER_HOME_BOARD_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_nut.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "on the shoulder_pitch home sensor's KY-003, on its M2s"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nuts: the geometry until a vendor model exists."""
    return pattern([hex_nut(HOME.board_nut, math.radians(CORNER_DEG), (x, y), KY003_BOARD_T) for x, y in POINTS])


@step(out="shoulder_home_board_nuts.step")
def shoulder_home_board_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    shoulder_home_board_nuts()   # build: writes the sibling shoulder_home_board_nuts.step
