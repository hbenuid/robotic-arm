"""nema17_40mm - purchased (COTS) part; source of truth is vendor/nema17_40mm.step.

NEMA 17 stepper, 40 mm body: the motor of the MKS SERVO42D closed-loop kit that drives the elbow_pitch and
wrist_pitch belt joints (lib/mounts.py places it on the NEMA 17 pads j1_link and j2_link carry; base_yaw takes
the 48 mm motor, parts/cycloidal/nema17_48mm). The vendor file is the motor body of the "nema17x40_with_mks" SolidWorks export,
split off by tools/reference/split_mks_motor.py (the board is parts/joints/mks_servo42d) and re-framed like
the drive motor (parts/cycloidal/nema17_48mm: mounting face z=0, body -Z, pilot boss and shaft +Z, D-flat
+Y), with the export's own boss and 23 mm shaft cut off and the drive's pilot() + shaft() (lib/cycloidal/motor.py,
MotorParams: Ø22 x 2, Ø5 x 22 with the 18 mm D-cut) fused on - every motor in the arm carries the same
interface. The cable connector is a 7 mm boss on the -Y side of the body's rear.

SolidWorks product: 'nema17x40_with_mks' (motor body + shaft)
Source export:      mks/nema17x40_with_mks.step (lib.reference.MKS_EXPORT_NAME)
Reference: mm units, 2 solid(s), bbox size (42, 49, 62.4) mm, bbox min (-21, -28, -40.4) mm.
In the arm: x3 (nema17_40mm#2 elbow_pitch, #3 wrist_pitch, one inside forearm_roll_drive#1 for forearm_roll).

COTS convention (parts/_templates/cots.py): nema17_40mm() returns the vendor STEP when present,
else the parametric envelope below - both in the part frame lib/mounts.py places.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import step

from lib.cots import hybrid
from lib.cycloidal import motor_bolt_points
from lib.cycloidal.motor import nema17_motor
from lib.datum import IDENTITY
from lib.geom import cylinder, single_solid
from lib.params import (
    MOTOR_40,
    NEMA17_40_CONNECTOR_D,
    NEMA17_40_CONNECTOR_W,
    NEMA17_40_CONNECTOR_Z0,
    NEMA17_40_CONNECTOR_Z1,
    NEMA17_40_MASS_G,
    NEMA17_40_REAR_STUB_DIA,
    NEMA17_40_REAR_STUB_LEN,
    NUDGE,
)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = NEMA17_40_MASS_G   # [ESTIMATE] see lib/params.py
PURCHASE_SPEC = "NEMA 17 stepper, 40 mm body, 5 mm D-shaft 22 mm (the motor of the MKS SERVO42D closed-loop kit, 17HS4401 class)"
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = ("ordered as MKS SERVO42D closed-loop kits (motor + board, parts/joints/mks_servo42d); the model carries the drive "
                 "motor's shaft (22 mm, 18 mm D-cut) - the kit export had 23 mm; confirm the shaft length and the mass on the unit in hand")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the part frame (identity: split_mks_motor.py writes the vendor file
# already re-framed; set it after swapping in a differently oriented catalog model).
VENDOR_TO_REF = IDENTITY

# The drive motor's parameters with the 40 mm body (same 31 mm bolt square, Ø22 x 2 pilot, Ø5 x 22 shaft) - lib/motors.py.
MOTOR = MOTOR_40


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing: the simplified motor + the rear
    bearing stub + the connector boss, at the vendor geometry's bounding box."""
    motor = nema17_motor(MOTOR, motor_bolt_points())
    stub = cylinder(NEMA17_40_REAR_STUB_DIA / 2.0, NEMA17_40_REAR_STUB_LEN + NUDGE, z0=-(MOTOR.body_length + NEMA17_40_REAR_STUB_LEN))
    connector = bd.Box(NEMA17_40_CONNECTOR_W, NEMA17_40_CONNECTOR_D + NUDGE, NEMA17_40_CONNECTOR_Z1 - NEMA17_40_CONNECTOR_Z0,
                       align=(bd.Align.CENTER, bd.Align.MIN, bd.Align.MIN)
                       ).moved(bd.Location((0.0, -(MOTOR.body_width / 2.0 + NEMA17_40_CONNECTOR_D), NEMA17_40_CONNECTOR_Z0)))
    return single_solid(motor + stub + connector)


@step
def nema17_40mm():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    nema17_40mm()   # build: writes the sibling nema17_40mm.step
