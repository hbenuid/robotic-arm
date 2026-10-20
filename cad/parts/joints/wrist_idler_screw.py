"""wrist_idler_screw - purchased (COTS) part: the wrist idler's axle, an M5 socket head cap screw (ISO 4762).

In the arm: one, under j2_link's web (lib/forearm/params.py IdlerParams; lib/mounts.py WRIST_IDLER_MOUNTS): its head's
bearing face on the idler's bottom face (gt2_idler_20t#3), the shank up through the idler, the rib under the web and the
web's slot into the nut held in the web's top (wrist_idler_nut); turned from below to clamp the idler where the belt is
tight. Length: IdlerParams.screw_len, its tip at the nut's top face.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_wrist_idler.py locks its numbers. Frame: axis on Z, the
head's bearing face on z=0 (the head in -Z), the shank in +Z.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import IdlerParams
from lib.params import WRIST_IDLER_SCREW_MASS_G

SCREW, LENGTH = IdlerParams().screw, IdlerParams().screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_IDLER_SCREW_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the wrist idler's axle: up from under the idler through the forearm's web into its nut"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain screw: the geometry until a vendor model exists."""
    return pattern([shcs(SCREW, LENGTH)])


@step
def wrist_idler_screw():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_idler_screw()   # build: writes the sibling wrist_idler_screw.step
