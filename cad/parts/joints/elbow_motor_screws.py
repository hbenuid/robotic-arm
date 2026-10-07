"""elbow_motor_screws - purchased (COTS) part: the elbow motor's 4 M3 socket head cap screws (ISO 4762).

They hold the elbow motor (nema17_40mm#2) down on the web across j1_link's motor hole.

In the arm (lib/mounts.py, hosted on j1_link#1): the heads on the web's underside (the belt's side, in the open lower
half of the hole), the shanks up through the web's M3 clearance holes into the motor's tapped holes,
J1_MOTOR_SCREW_THREAD of thread in them. Length: lib/params.py J1_MOTOR_SCREW_LEN (the web, ArmParams.motor_plate_t,
and that thread).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/upper_arm/test_elbow_motor_screws.py locks its numbers. Frame: axis
on Z, the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the web's holes
(lib/upper_arm/layout.py pad_holes() as (x, y) = j1_link's (x, -z) about the pad's axis - the mount turns +Z up j1_link's
+Y).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import ELBOW_MOTOR_SCREWS_MASS_G, J1_MOTOR_SCREW_LEN, M3_SHCS
from lib.upper_arm import DEFAULT as UPPER_ARM
from lib.upper_arm import pad_holes

LENGTH = J1_MOTOR_SCREW_LEN
POINTS = tuple((x, -z) for x, z, _ in pad_holes(UPPER_ARM))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ELBOW_MOTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M3_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "the elbow motor onto the web across j1_link's motor hole, up through it into the motor's tapped holes"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M3_SHCS, LENGTH, xy) for xy in POINTS)


@step
def elbow_motor_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_motor_screws()   # build: writes the sibling elbow_motor_screws.step
