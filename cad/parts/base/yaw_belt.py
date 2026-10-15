"""yaw_belt - purchased (COTS) part: the base_yaw belt, a closed GT2 (2GT) belt, 6 mm, YAW_BELT long - one of the
multipack lengths (lib/belts.py STANDARD_2GT_LENGTHS).

It runs from the 48 mm motor's 20T (gt2_pulley_20t#3) on the base motor mount to the 120T under j1_coupler, round
its tensioner on the mount's plate (gt2_idler_20t#2, lib/base/params.py YawIdlerParams): the motor sits YAW_MOTOR_X
out, short of the stock belt's own centre distance, and the idler takes up the rest.

No catalog model, no export and NO reference file: a purchased part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry: a smooth band round its loop (lib/pulley/belt.py closed_belt(); no
teeth), and tests/test_belts.py locks its numbers. Frame (lib/base/layout.py yaw_belt_loop()): the motor's 20T on the
origin, the 120T on +X, the idler on the run back to the 20T, the band centred on z=0; the arm places it by
yaw_belt_frame() (lib/mounts.py BELT_MOUNTS).
"""
import pathlib

from cadgen import step

from lib.base import yaw_belt_loop
from lib.base.params import YAW_BELT
from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.params import GT2_BELT_MASS_G_PER_M, GT2_BELT_W
from lib.pulley.belt import closed_belt

LENGTH = YAW_BELT

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = LENGTH * GT2_BELT_MASS_G_PER_M / 1000.0   # [ESTIMATE] lib/belts.py
PURCHASE_SPEC = f"{LENGTH}-2GT closed GT2 belt, {GT2_BELT_W:g} mm"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the base_yaw belt: a GT2 multipack length, tensioned by its idler on the motor mount"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def loop():
    """The belt's loop in its own frame (lib/belts.py)."""
    return yaw_belt_loop()


def _envelope():
    """The smooth band round the loop: the geometry until a vendor model exists."""
    return closed_belt(loop())


@step
def yaw_belt():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_belt()   # build: writes the sibling yaw_belt.step
