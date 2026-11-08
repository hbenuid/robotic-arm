"""elbow_motor_screws - purchased (COTS) part: the elbow motor's 4 M3 socket head cap screws (ISO 4762).

They hold the elbow motor (nema17_40mm#2) on its gearbox's input end (eg17_g5_gearbox#1): the 4 that ship with the
gearbox, from its front through its input plate into the motor's tapped holes.

In the arm (lib/mounts.py, hosted on eg17_g5_gearbox#1): the heads on the input plate's front face in the gearbox's
corner channels (EG17_G5.channel), the shanks through the plate's holes on the NEMA 17 square (EG17_G5.plate_sp,
plate_hole) into the motor. Length (under the head): lib/params.py J1_MOTOR_SCREW_LEN (the plate, EG17_G5.plate_t,
and the thread in the motor).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/upper_arm/test_elbow_motor_screws.py locks its numbers. Frame: axis
on Z, the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the NEMA 17 square about the axis
(the mount turns +Z down the gearbox's -Z, into the motor).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import EG17_G5, ELBOW_MOTOR_SCREWS_MASS_G, J1_MOTOR_SCREW_LEN, M3_SHCS

SCREW = M3_SHCS
LENGTH = J1_MOTOR_SCREW_LEN
POINTS = tuple((sx * EG17_G5.plate_sp / 2.0, sy * EG17_G5.plate_sp / 2.0) for sx, sy in ((1, -1), (1, 1), (-1, 1), (-1, -1)))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ELBOW_MOTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the elbow motor onto its gearbox, through the gearbox's input plate into the motor's tapped holes - the 4 that "
                 "ship with the EG17-G5 (their length the model's guess: confirm on the unit)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="elbow_motor_screws.step")
def elbow_motor_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_motor_screws()   # build: writes the sibling elbow_motor_screws.step
