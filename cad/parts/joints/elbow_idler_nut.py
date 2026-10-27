"""elbow_idler_nut - purchased (COTS) part: the elbow belt tensioner's M5 hex nut (ISO 4032), in j1_link's slab.

In the arm: one, on j1_link (lib/upper_arm/params.py ElbowIdlerParams; lib/mounts.py IDLER_MOUNTS): dropped into its slot
down from the slab's top face, its bearing face on the slot's floor (ElbowIdlerParams.floor over the underside), its
flats across the link's X against the slot's walls - it slides along Z with the tensioner's axle (elbow_idler_screw) and
cannot turn.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/upper_arm/test_elbow_idler.py locks its numbers.
Frame: axis on Z, standing on z=0, a corner on +Y (the flats across x).
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import ELBOW_IDLER_NUT_MASS_G
from lib.upper_arm import ElbowIdlerParams

NUT = ElbowIdlerParams().nut
CORNER_DEG = 90.0   # a corner on +Y: the flats face +/- X

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ELBOW_IDLER_NUT_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "in the slot down from j1_link's top face, over the elbow belt's idler: drop it in before the screw goes up"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nut, a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern([hex_nut(NUT, math.radians(CORNER_DEG))])


@step(out="elbow_idler_nut.step")
def elbow_idler_nut():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_idler_nut()   # build: writes the sibling elbow_idler_nut.step
