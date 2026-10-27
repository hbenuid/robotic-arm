"""wrist_motor_screws - purchased (COTS) part: the wrist-pitch motor's 4 M3 socket head cap screws (ISO 4762).

They hold the wrist-pitch motor (nema17_40mm#3) down on j2_link's web - the elbow motor's screws (elbow_motor_screws)
in the elbow motor's seat: the same 11 mm web, the same counterbores, the same thread in the motor.

In the arm (lib/mounts.py, hosted on j2_link#1): the heads flush in the counterbores in the web's underside (the belt
side; lib/forearm/params.py MotorHoleParams.screw, pocket_dia), the shanks up through the web's M3 clearance holes into
the motor's tapped holes, MotorHoleParams.thread of thread in them. Length (under the head): lib/params.py
J2_MOTOR_SCREW_LEN (the web under the heads and that thread). Fit them before the wrist belt's idler: its flange
passes under the -x +y head (its socket clear).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_wrist_motor_screws.py locks its numbers. Frame: axis
on Z, the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the web's holes about the motor's
axis (lib/forearm/layout.py motor_hole_points() less motor_x - the mount puts the origin on that axis, +Z up j2_link's
+Z).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import motor_hole_points
from lib.params import J2_MOTOR_SCREW_LEN, WRIST_MOTOR_SCREWS_MASS_G

SCREW = FOREARM.holes.screw
LENGTH = J2_MOTOR_SCREW_LEN
POINTS = tuple((x - FOREARM.motor_x, y) for x, y in motor_hole_points(FOREARM))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_MOTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the wrist-pitch motor onto j2_link's web, the heads flush in its counterbores, up into the motor's "
                 "tapped holes")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="wrist_motor_screws.step")
def wrist_motor_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_motor_screws()   # build: writes the sibling wrist_motor_screws.step
