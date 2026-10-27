"""roll_idler_screw - purchased (COTS) part: the roll belt tensioner's axle, an M5 socket head cap screw (ISO 4762).

In the arm: one, in forearm_roll_drive (lib/forearm/params.py RollIdlerParams): its head's bearing face on the idler's
top (gt2_idler_20t), the shank down through the idler, the riser on the roll frame's front face and its slot into the
nut in the channel under the face (roll_idler_nut); slid along the slot to tension the roll belt, then turned from
above to clamp the idler there. Length: RollIdlerParams.screw_len, its tip 2 pitches and more past the nut.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_roll_idler.py locks its numbers. Frame: axis on Z, the
head's bearing face on z=0 (the head in -Z), the shank in +Z.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import RollIdlerParams
from lib.params import ROLL_IDLER_SCREW_MASS_G

SCREW, LENGTH = RollIdlerParams().screw, RollIdlerParams().screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_IDLER_SCREW_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the roll belt tensioner's axle: down through the idler and the roll frame's riser into its nut"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screw: the geometry until a vendor model exists."""
    return pattern([shcs(SCREW, LENGTH)])


@step(out="roll_idler_screw.step")
def roll_idler_screw():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_idler_screw()   # build: writes the sibling roll_idler_screw.step
