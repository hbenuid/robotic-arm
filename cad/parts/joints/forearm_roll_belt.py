"""forearm_roll_belt - purchased (COTS) part: the forearm roll's belt, a closed GT2 (2GT) belt, 6 mm,
FOREARM_ROLL_BELT_LENGTH long - one of the multipack lengths (lib/belts.py STANDARD_2GT_LENGTHS).

It runs in front of the roll frame from the roll motor's 20T (on the elbow axis) to the pulley's 90T ring; its centre
distance is the arm's elbow offset (lib/params.py ELBOW_ROLL_OFFSET); the frame plate's slots tension it.

No catalog model, no export and NO reference file: a purchased part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry: a smooth band round its loop (lib/pulley/belt.py closed_belt(); no
teeth), and tests/test_belts.py locks its numbers. Frame (lib/forearm/layout.py roll_belt_loop()): the motor's 20T on
the origin, the ring on +X, the band centred on z=0; a row of assemblies/forearm_roll_drive.py places it.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.forearm import roll_belt_loop
from lib.params import FOREARM_ROLL_BELT_LENGTH, GT2_BELT_MASS_G_PER_M, GT2_BELT_W
from lib.pulley.belt import closed_belt

LENGTH = FOREARM_ROLL_BELT_LENGTH

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = LENGTH * GT2_BELT_MASS_G_PER_M / 1000.0   # [ESTIMATE] lib/belts.py
PURCHASE_SPEC = f"{LENGTH}-2GT closed GT2 belt, {GT2_BELT_W:g} mm"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = "the forearm roll's belt: a GT2 multipack length; the roll frame plate's slots tension it"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def loop():
    """The belt's loop in its own frame (lib/belts.py)."""
    return roll_belt_loop()


def _envelope():
    """The smooth band round the loop: the geometry until a vendor model exists."""
    return closed_belt(loop())


@step
def forearm_roll_belt():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    forearm_roll_belt()   # build: writes the sibling forearm_roll_belt.step
