"""nema17_48mm - NEMA 17 stepper, 48 mm body, 5 mm D-shaft (the drive's input motor).

Frame: 42.3 mm square body in -Z, 22 mm pilot boss and the 22 mm shaft (4 mm round + 18 mm D-cut, flat
on +Y) in +Z, 4x M3 blind holes on the 31 mm square. Mounting face at z=0 - the drive's stack
datum. reference/nema17_48mm.step is the drive repo's simplified builder export (cycloidal_drive@2f1f67d
src/purchased_parts.py build_nema17_motor, kind "cots") - the envelope below reproduces it.
vendor/nema17_48mm.step (the geometry the drive shows) is composed by tools/reference/split_mks_motor.py
from the user's SolidWorks kit export: the x48 export's real 48 mm body (plates, housing, bearings,
connector, rotor - 7 solids, the tie rods left out because the MKS kit's M3x30 replace them) with its
own boss and shaft cut off at the mounting face and THIS motor's pilot() + shaft() (lib/cycloidal/motor.py,
the same MotorParams as the envelope) fused on, already in this frame (VENDOR_TO_REF identity). The
datasheet 48 mm motor (17HS19-2004S1) ships a 24 mm shaft with a 15 mm D-cut - the user's motor is the
22 mm one; measure before ordering.
In the arm: x2 - inside cycloidal_drive#1 and, mounted under the base plate, nema17_48mm#1 (lib/mounts.py, base_yaw).
Not the wrist's pancake motor (parts/nema17_pancake.py) nor the elbow / wrist-pitch 40 mm motors
(parts/joints/nema17_40mm.py - same builder, lib/cycloidal/motor.py, other MotorParams). The MKS SERVO42D
board on its rear face is its own part (parts/joints/mks_servo42d.py, placed by assemblies/cycloidal_drive.py).
"""
import pathlib

from cadgen import step
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, motor_bolt_points
from lib.cycloidal.motor import nema17_motor
from lib.datum import IDENTITY
from lib.params import CYCLOIDAL_MOTOR_MASS_G
from parts.cycloidal._cots import hybrid

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_MOTOR_MASS_G
PURCHASE_SPEC = "NEMA 17 stepper, {0.body_length:g} mm body, {0.shaft_dia:g} mm D-shaft {0.shaft_length:g} mm long (17HS19-2004S1 class)".format(DEFAULT_CONFIG.motor)
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = ("ordered as the MKS SERVO42D closed-loop kit with the 48 mm motor (the board is parts/joints/mks_servo42d); "
                 "check the shaft length from the mounting face before ordering: shorter catalog shafts leave too little D-bore engagement")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY   # set after inspecting a step.parts model (see vendor/README.md)


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    return nema17_motor(cfg.motor, motor_bolt_points(cfg))


@step
def nema17_48mm():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    nema17_48mm()   # build: writes the sibling nema17_48mm.step (preview: ./cadtool show parts/cycloidal/nema17_48mm.py)
