"""elbow_belt - purchased (COTS) part: the elbow belt, a closed GT2 (2GT) belt, 6 mm, ELBOW_BELT_LENGTH long - one of
the multipack lengths (lib/belts.py STANDARD_2GT_LENGTHS).

It runs under j1_link's web from the elbow motor's 20T (gt2_pulley_20t#2) to the elbow 108T, round its tensioner: the
stock length is longer than the open loop at ELBOW_MOTOR_CENTRES, and a GT2 idler on its back (gt2_idler_20t#1, under
j1_link's slab - lib/upper_arm/params.py ElbowIdlerParams) takes up the rest, slid along the slab's slots.

No catalog model, no export and NO reference file: a purchased part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry: a smooth band round its loop (lib/pulley/belt.py closed_belt(); no
teeth), and tests/test_belts.py locks its numbers. Frame (lib/upper_arm/layout.py elbow_belt_loop()): the motor's 20T on
the origin, the 108T on +X, the idler on the +Y run, the band centred on z=0; the arm places it on j1_link
(lib/mounts.py BELT_MOUNTS).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.params import ELBOW_BELT_LENGTH, GT2_BELT_MASS_G_PER_M, GT2_BELT_W
from lib.pulley.belt import closed_belt
from lib.upper_arm import elbow_belt_loop

LENGTH = ELBOW_BELT_LENGTH

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = LENGTH * GT2_BELT_MASS_G_PER_M / 1000.0   # [ESTIMATE] lib/belts.py
PURCHASE_SPEC = f"{LENGTH}-2GT closed GT2 belt, {GT2_BELT_W:g} mm"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the elbow belt: a GT2 multipack length"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def loop():
    """The belt's loop in its own frame (lib/belts.py)."""
    return elbow_belt_loop()


def _envelope():
    """The smooth band round the loop: the geometry until a vendor model exists."""
    return closed_belt(loop())


@step
def elbow_belt():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_belt()   # build: writes the sibling elbow_belt.step
