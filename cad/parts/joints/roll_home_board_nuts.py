"""roll_home_board_nuts - purchased (COTS) part: the 2 M2 hex nuts (ISO 4032) on the forearm_roll home sensor's KY-003.

In the arm: one pattern (roll_home_board_nuts, a row of forearm_roll_drive#1 in the KY-003's frame - lib/forearm/roll.py
home_sensor_frame()): on the board's component side at its two holes, on the M2s up through the carrier's plate
(roll_home_board_screws).

No catalog model (none adopted), no SolidWorks export and NO reference file: a pattern part with no reference
(lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut()) and
tests/forearm/test_roll_home.py locks its numbers. Frame: the KY-003's, the nuts on the board's top (z = KY003_BOARD_T).
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import DEFAULT
from lib.forearm.roll import home_board_points
from lib.params import KY003_BOARD_T, ROLL_HOME_BOARD_NUTS_MASS_G

HOME = DEFAULT.drive.home
POINTS = home_board_points(DEFAULT)
CORNER_DEG = 90.0   # a corner on +Y: the flats face +/- X

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_HOME_BOARD_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{HOME.board_nut.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence
PURCHASE_NOTE = "on the forearm_roll home sensor's KY-003, on its M2s"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nuts: the geometry until a vendor model exists."""
    return pattern([hex_nut(HOME.board_nut, math.radians(CORNER_DEG), (x, y), KY003_BOARD_T) for x, y in POINTS])


@step(out="roll_home_board_nuts.step")
def roll_home_board_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_home_board_nuts()   # build: writes the sibling roll_home_board_nuts.step
