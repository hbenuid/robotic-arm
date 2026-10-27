"""yaw_idler_nut - purchased (COTS) part: the base_yaw belt tensioner's M5 hex nut (ISO 4032), under the motor mount's
plate.

In the arm: one, on the base's motor mount (lib/base/params.py YawIdlerParams; lib/mounts.py YAW_IDLER_MOUNTS): between
the two rails under the mount's plate, its bearing face on the plate's underside, its flats against the rails - it
slides across the belt with the tensioner's axle (yaw_idler_screw) and cannot turn.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/base/test_yaw_idler.py locks its numbers.
Frame: axis on Z, standing on z=0, a corner on +Y (the flats across x).
"""
import math
import pathlib

from cadgen import step

from lib.base import YawIdlerParams
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import YAW_IDLER_NUT_MASS_G

NUT = YawIdlerParams().nut
CORNER_DEG = 90.0   # a corner on +Y: the flats face +/- X

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_IDLER_NUT_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "between the rails under the base motor mount's plate, under the base_yaw belt's idler: hold it up as the screw goes down"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain nut, a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern([hex_nut(NUT, math.radians(CORNER_DEG))])


@step(out="yaw_idler_nut.step")
def yaw_idler_nut():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_idler_nut()   # build: writes the sibling yaw_idler_nut.step
