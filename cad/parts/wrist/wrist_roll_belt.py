"""wrist_roll_belt - purchased (COTS) part: the wrist roll's belt, a closed GT2 (2GT) belt, 6 mm,
WRIST_ROLL_BELT_LENGTH long - a stock length (lib/belts.py STANDARD_2GT_LENGTHS).

It runs in front of wrist_link's frame from the pancake's 20T (on the wrist-pitch axis) to the roll pulley's 40T band,
2 : 1 (WRIST_ROLL_RATIO), at the stock length's centre distance (WRIST_ROLL_OFFSET, the arm's wrist offset) with the
pancake at its slots' middle; the slots (+/- 2.5 along the belt) tension it - no idler yet (docs/open_issues.md).

No catalog model, no export and NO reference file: a purchased part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry: a smooth band round its loop (lib/pulley/belt.py closed_belt(); no
teeth), and tests/test_belts.py locks its numbers. Frame (lib/wrist/layout.py roll_belt_loop()): the pancake's 20T on the
origin, the 40T on +X, the band centred on z=0; a row of assemblies/wrist_roll_drive.py places it.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.params import GT2_BELT_MASS_G_PER_M, GT2_BELT_W, WRIST_ROLL_BELT_LENGTH
from lib.pulley.belt import closed_belt
from lib.wrist.layout import roll_belt_loop

LENGTH = WRIST_ROLL_BELT_LENGTH

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = LENGTH * GT2_BELT_MASS_G_PER_M / 1000.0   # [ESTIMATE] lib/belts.py
PURCHASE_SPEC = f"{LENGTH}-2GT closed GT2 belt, {GT2_BELT_W:g} mm"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the wrist roll's belt: a stock GT2 length, tensioned by the pancake's slots"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def loop():
    """The belt's loop in its own frame (lib/belts.py)."""
    return roll_belt_loop()


def _envelope():
    """The smooth band round the loop: the geometry until a vendor model exists."""
    return closed_belt(loop())


@step(out="wrist_roll_belt.step")
def wrist_roll_belt():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_belt()   # build: writes the sibling wrist_roll_belt.step
