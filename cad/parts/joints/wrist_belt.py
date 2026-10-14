"""wrist_belt - purchased (COTS) part: the wrist-pitch belt, a closed GT2 (2GT) belt, 6 mm, WRIST_BELT_LENGTH long -
one of the multipack lengths (lib/belts.py STANDARD_2GT_LENGTHS).

It runs under j2_link's web from the wrist motor's 20T (gt2_pulley_20t#4) to the wrist 90T: the motor sits at the
stock length's centre distance (J2_MOTOR_SLIDE_X), no idler; sliding the motor along its slots tensions it.

No catalog model, no export and NO reference file: a purchased part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry: a smooth band round its loop (lib/pulley/belt.py closed_belt(); no
teeth), and tests/test_belts.py locks its numbers. Frame (lib/forearm/layout.py wrist_belt_loop()): the motor's 20T on
the origin, the 90T on +X (an idler between them on the run leaving the 20T when ForearmConfig.idler is set), the
band centred on z=0; the arm places it by
wrist_belt_frame() (lib/mounts.py BELT_MOUNTS).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.forearm import wrist_belt_loop
from lib.params import GT2_BELT_MASS_G_PER_M, GT2_BELT_W, WRIST_BELT_LENGTH
from lib.pulley.belt import closed_belt

LENGTH = WRIST_BELT_LENGTH

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = LENGTH * GT2_BELT_MASS_G_PER_M / 1000.0   # [ESTIMATE] lib/belts.py
PURCHASE_SPEC = f"{LENGTH}-2GT closed GT2 belt, {GT2_BELT_W:g} mm"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the wrist-pitch belt: a GT2 multipack length, under the forearm's web (no idler: the motor's slots tension it)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def loop():
    """The belt's loop in its own frame (lib/belts.py)."""
    return wrist_belt_loop()


def _envelope():
    """The smooth band round the loop: the geometry until a vendor model exists."""
    return closed_belt(loop())


@step
def wrist_belt():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_belt()   # build: writes the sibling wrist_belt.step
