"""wrist_idler_nut - purchased (COTS) part: the wrist idler's M5 hex nut (ISO 4032), in j2_link's web.

NOT PLACED (UNPLACED below): the wrist belt has no idler. With one (lib/forearm/params.py IdlerParams, a
ForearmConfig.idler) it sits on the floor of its channel down from the web's top face (IdlerParams.nut_floor), its flats
across the forearm's x against the channel's walls - it slides along y with the idler's axle (wrist_idler_screw) and
cannot turn.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/forearm/test_wrist_idler.py locks its numbers.
Frame: axis on Z, the bearing face on z=0, standing in +Z, a corner on +Y (the flats across x).
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import IdlerParams
from lib.params import WRIST_IDLER_NUT_MASS_G

NUT = IdlerParams().nut
CORNER_DEG = 90.0   # a corner on +Y: the flats face +/- X

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_IDLER_NUT_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "in the channel in the forearm web's top, under the wrist motor's edge: drop it in before the motor"
UNPLACED = "the wrist belt has no idler (its motor at the stock belt's centre distance): a ForearmConfig.idler places it again"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nut, a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern([hex_nut(NUT, math.radians(CORNER_DEG))])


@step
def wrist_idler_nut():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_idler_nut()   # build: writes the sibling wrist_idler_nut.step
