"""roll_motor_screws - purchased (COTS) part: the forearm roll motor's 4 M3 socket head cap screws (ISO 4762).

They hold the roll motor (nema17_40mm:forearm_roll) on the roll frame's plate (forearm_roll_block) - the elbow and the
wrist-pitch motors' screw head and counterbore (lib/forearm/params.py MotorHoleParams), through a thinner plate: the
idler on the frame's face tensions the roll belt, so the motor sits on round holes and does not slide.

In the arm: x1, in forearm_roll_drive (its row in assemblies/forearm_roll_drive.py, the stator - elbow_link): the heads
flush in the counterbores in the frame's front face (RollDriveParams.motor_holes screw, pocket_dia), the shanks down
through the plate's M3 clearance holes into the motor's tapped holes, MotorHoleParams.thread of thread in them. Length
(under the head): lib/params.py ROLL_MOTOR_SCREW_LEN (the plate under the heads and that thread). Fit them once the
motor is lifted onto the plate, from the front face (the 20T clear inside them).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_roll_motor_screws.py locks its numbers. Frame: axis
on Z, the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the plate's holes about the
motor's axis (lib/forearm/layout.py pad_bolt_points() less the motor's axis - the module's row puts the origin on that
axis, turned over: +Z down the roll axis into the motor).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import pad_bolt_points, stack_positions
from lib.params import ROLL_MOTOR_SCREW_LEN, ROLL_MOTOR_SCREWS_MASS_G

SCREW = FOREARM.drive.motor_holes.screw
LENGTH = ROLL_MOTOR_SCREW_LEN
_S = stack_positions(FOREARM)
POINTS = tuple((x - _S["x_motor"], y - _S["y_motor"]) for x, y in pad_bolt_points(FOREARM))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_MOTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the forearm roll motor onto the roll frame's plate, the heads flush in its front face's counterbores, "
                 "into the motor's tapped holes")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step
def roll_motor_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_motor_screws()   # build: writes the sibling roll_motor_screws.step
