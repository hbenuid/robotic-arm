"""elbow_idler_screw - purchased (COTS) part: the elbow belt tensioner's axle, an M5 socket head cap screw (ISO 4762).

In the arm: one, on j1_link (lib/upper_arm/params.py ElbowIdlerParams; lib/mounts.py IDLER_MOUNTS): its head's bearing
face on the idler's bottom face (gt2_idler_20t#1), the shank up through the idler, the spacer (elbow_idler_spacer) and
the slot through the slab's floor into the nut in the slot down from the slab's top face (elbow_idler_nut); slid along
the slots to tension the elbow belt, then turned from below to clamp the idler there. Length:
ElbowIdlerParams.screw_len, its tip 2 pitches past the nut.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/upper_arm/test_elbow_idler.py locks its numbers. Frame: axis on Z,
the head's bearing face on z=0 (the head in -Z), the shank in +Z.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import ELBOW_IDLER_SCREW_MASS_G
from lib.upper_arm import ElbowIdlerParams

SCREW, LENGTH = ElbowIdlerParams().screw, ElbowIdlerParams().screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ELBOW_IDLER_SCREW_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the elbow belt tensioner's axle: up through the idler, its spacer and j1_link's slot into its nut"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screw: the geometry until a vendor model exists."""
    return pattern([shcs(SCREW, LENGTH)])


@step
def elbow_idler_screw():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_idler_screw()   # build: writes the sibling elbow_idler_screw.step
