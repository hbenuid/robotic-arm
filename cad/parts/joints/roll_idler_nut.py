"""roll_idler_nut - purchased (COTS) part: the roll belt tensioner's M5 hex nut (ISO 4032), in the roll frame.

In the arm: one, in forearm_roll_drive (lib/forearm/params.py RollIdlerParams): in its channel under the roll frame's
front face, pushed in from the frame's open +N face, its bearing face up against the channel's roof, its flats across
the frame's y against the channel's walls - it slides along x with the tensioner's axle (roll_idler_screw) and cannot
turn.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/forearm/test_roll_idler.py locks its numbers.
Frame: axis on Z, standing on z=0, a corner on +X (the flats across y).
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import RollIdlerParams
from lib.params import ROLL_IDLER_NUT_MASS_G

NUT = RollIdlerParams().nut
CORNER_DEG = 0.0   # a corner on +X: the flats face +/- Y

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_IDLER_NUT_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "in the channel under the roll frame's front face: push it in from the frame's open +N face before the idler goes on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nut, a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern([hex_nut(NUT, math.radians(CORNER_DEG))])


@step
def roll_idler_nut():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_idler_nut()   # build: writes the sibling roll_idler_nut.step
