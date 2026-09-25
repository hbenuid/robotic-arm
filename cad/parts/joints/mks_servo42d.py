"""mks_servo42d - purchased (COTS) part; source of truth is vendor/mks_servo42d.step.

The MKS SERVO42D closed-loop driver board as it ships bolted to a NEMA 17's rear face: PCB + cover
(43 x 43 x 11.1) on 4x 7 x 3 standoffs, held by 4x M3x30 through the motor body. One occurrence
behind every MKS motor of the arm: the three 40 mm motors (parts/joints/nema17_40mm, placed by
lib/mounts.py) and the cycloidal drive's 48 mm motor (assemblies/cycloidal_drive.py, stack_positions
"z_mks_board"). The vendor file is the Servo42D sub-assembly + standoffs + screws of the
"nema17x40_with_mks" SolidWorks export, split off by tools/reference/split_mks_motor.py.

Frame: z=0 at the MOTOR's rear face, the standoffs + board in -Z (cover face at z=-14.1), the screws
reaching to z=+19.6 into the motor's through-holes; the board square is centred on the motor axis
and its edges are parallel to the motor's (the connector side of the PCB on -Y, like the motor's).

SolidWorks product: 'nema17x40_with_mks' (Servo42D_Assem + standoffs + screws)
Source export:      ~/Documents/arm_assembly_organized/mks/nema17x40_with_mks.step (lib.reference.MKS_EXPORT_NAME)
Reference: mm units, 13 solid(s), bbox size (43, 43, 33.7) mm, bbox min (-21.5, -21.5, -14.1) mm.
In the arm: x5 (mks_servo42d#1 behind nema17_48mm#1 under the base, #2..3 behind nema17_40mm#2..3, one inside cycloidal_drive#1, one inside forearm_roll_drive#1).

COTS convention (parts/_templates/cots.py): mks_servo42d() returns the vendor STEP when present,
else the parametric envelope below - both in the frame above.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import step

from lib.cots import hybrid
from lib.cycloidal import DEFAULT_CONFIG, motor_bolt_points
from lib.datum import IDENTITY
from lib.geom import cylinder, single_solid
from lib.params import MKS_SERVO42D_MASS_G, MKS_SERVO42D_SCREW_REACH, MKS_SERVO42D_STACK, MKS_SERVO42D_W

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = MKS_SERVO42D_MASS_G   # [ESTIMATE] see lib/params.py
PURCHASE_SPEC = "MKS SERVO42D closed-loop stepper driver board + cover, with 4x M3x30 and 4x 3 mm standoffs (the kit's board half)"
PURCHASE_QTY = 1    # pieces per occurrence: one board kit behind one motor
PURCHASE_NOTE = ("ordered as MKS SERVO42D closed-loop kits (motor + board): 3 kits with a 40 mm motor (parts/joints/nema17_40mm, "
                 "elbow + wrist pitch) + 2 kits with the 48 mm motor (parts/cycloidal/nema17_48mm: base_yaw and the drive); "
                 "CAN ids software/control/src/config.py J1..J3")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the part frame (identity: split_mks_motor.py writes the vendor file
# already re-framed; set it after swapping in a differently oriented catalog model).
VENDOR_TO_REF = IDENTITY

SCREW_HEAD_Z = MKS_SERVO42D_STACK - 1.33   # [REFERENCE] the M3x30 heads sit 1.33 mm inside the cover face (screw z -12.77..+19.6)


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing: the cover box on the motor's rear
    face + the four screws, at the vendor geometry's bounding box."""
    cover = bd.Box(MKS_SERVO42D_W, MKS_SERVO42D_W, MKS_SERVO42D_STACK, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
    result = cover
    for xy in motor_bolt_points(DEFAULT_CONFIG):
        result = result + cylinder(DEFAULT_CONFIG.motor.bolt_dia / 2.0, MKS_SERVO42D_SCREW_REACH + SCREW_HEAD_Z, xy, z0=-SCREW_HEAD_Z)
    return single_solid(result)


@step
def mks_servo42d():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    mks_servo42d()   # build: writes the sibling mks_servo42d.step
