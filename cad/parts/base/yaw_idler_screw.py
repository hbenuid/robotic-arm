"""yaw_idler_screw - purchased (COTS) part: the base_yaw belt tensioner's axle, an M5 socket head cap screw (ISO 4762).

In the arm: one, on the base's motor mount (lib/base/params.py YawIdlerParams; lib/mounts.py YAW_IDLER_MOUNTS): its
head's bearing face on the idler's top face (gt2_idler_20t#2), the shank down through the idler, the spacer
(yaw_idler_spacer) and the slot through the mount's plate into the nut between the rails under it (yaw_idler_nut);
slid along the slot to tension the base_yaw belt, then turned from above to clamp the idler there. Length:
YawIdlerParams.screw_len, its tip 2 pitches past the nut.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_yaw_idler.py locks its numbers. Frame: axis on Z, the
head's bearing face on z=0 (the head in -Z), the shank in +Z.
"""
import pathlib

from cadgen import step

from lib.base import YawIdlerParams
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import YAW_IDLER_SCREW_MASS_G

SCREW, LENGTH = YawIdlerParams().screw, YawIdlerParams().screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_IDLER_SCREW_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the base_yaw belt tensioner's axle: down through the idler, its spacer and the motor mount's slot into its nut"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screw: the geometry until a vendor model exists."""
    return pattern([shcs(SCREW, LENGTH)])


@step
def yaw_idler_screw():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_idler_screw()   # build: writes the sibling yaw_idler_screw.step
