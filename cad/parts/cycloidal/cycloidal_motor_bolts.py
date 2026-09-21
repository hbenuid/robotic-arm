"""cycloidal_motor_bolts - the 4 M3 x 10 socket-head screws holding the NEMA 17 to the motor plate (thread tip at z=0, head on top).

In the drive: at stack z_motor_bolts (-5) - heads in the motor plate's inner-face pockets, threads
into the motor's blind holes. Plain cylinders (no thread, no socket).
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_motor_bolts.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_motor_bolts.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
import pathlib

from cadgen import step
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, motor_bolt_points
from lib.cycloidal.geom import cylinder
from lib.datum import IDENTITY
from lib.params import CYCLOIDAL_MOTOR_BOLTS_MASS_G
from parts.cycloidal._cots import hybrid, pattern

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_MOTOR_BOLTS_MASS_G
PURCHASE_SPEC = "M{0.bolt_dia:g} x {0.motor_bolt_thread_length:g} socket head cap screw (ISO 4762)".format(DEFAULT_CONFIG.motor)
PURCHASE_QTY = len(motor_bolt_points())    # pieces per occurrence (the whole pattern)
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    m = cfg.motor
    return pattern(cylinder(m.bolt_dia / 2.0, m.motor_bolt_thread_length, xy) + cylinder(m.motor_bolt_head_dia / 2.0, m.motor_bolt_head_height, xy, z0=m.motor_bolt_thread_length)
                   for xy in motor_bolt_points(cfg))


@step
def cycloidal_motor_bolts():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    cycloidal_motor_bolts()   # build: writes the sibling cycloidal_motor_bolts.step (preview: ./cadtool show parts/cycloidal/cycloidal_motor_bolts.py)
