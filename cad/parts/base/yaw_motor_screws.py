"""yaw_motor_screws - purchased (COTS) part: the base_yaw motor's 4 M3 socket head cap screws (ISO 4762).

They hold the base_yaw motor (nema17_48mm#1) up under the motor mount's plate - the wrist and roll motors' screws in
their seat (lib/forearm/params.py MotorHoleParams): the same counterbores, the same thread in the motor.

In the arm (lib/mounts.py, hosted on base_motor_mount#1): the heads flush in the counterbores in the plate's top
(lib/base/params.py MotorParams.holes: pocket_dia, the screw's head_h deep), the shanks down through the plate's and
its pad's M3 clearance holes into the motor's tapped holes, holes.thread of thread in them. Length (under the head):
lib/params.py YAW_MOTOR_SCREW_LEN (the plate and the pad under the heads, and that thread) - the roll motor's M3 x 8.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_yaw_motor_screws.py locks its numbers. Frame: axis on Z,
the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the plate's holes about the motor's axis
(lib/base/layout.py motor_holes() less the centre, the base's (x, z) as (x, y) - the mount puts the origin on that
axis, +Z down the base's -Y).
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT as BASE
from lib.base import motor_holes
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import YAW_MOTOR_SCREW_LEN, YAW_MOTOR_SCREWS_MASS_G

SCREW = BASE.motor.holes.screw
LENGTH = YAW_MOTOR_SCREW_LEN
POINTS = tuple((x - BASE.motor.centre[0], z - BASE.motor.centre[1]) for x, z in motor_holes(BASE))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_MOTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the base_yaw motor up under the motor mount's plate, the heads flush in its counterbores, down into "
                 "the motor's tapped holes")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="yaw_motor_screws.step")
def yaw_motor_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_motor_screws()   # build: writes the sibling yaw_motor_screws.step
