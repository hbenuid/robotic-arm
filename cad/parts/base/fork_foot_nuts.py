"""fork_foot_nuts - purchased (COTS) part: the 2 M4 hex nuts (ISO 4032) in j1_coupler's disc, for the screws that hold
j1_motor_leg's foot to it.

In the arm (lib/mounts.py FORK_MOUNTS, hosted on fork_foot_screws): each nut against the outer wall of its slot in the
disc (ForkParams.motor_nut_wall in from the leg's outer face: lib/yaw_coupler/layout.py fork_foot_nut_z), its flats
across the slot (the coupler's z), a corner up it - the slots are NUT_SLOT over the nut's AF and height, a slip fit.
They drop into the slots from the disc's top face before the motor leg slides on.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/base/test_fork_bolts.py locks its numbers. Frame:
fork_foot_screws' - axis on Z, the nuts' bearing faces on z=0, standing in +Z; the two at lib/yaw_coupler/layout.py
fork_foot_screw_points(), a corner along +Y (CORNER_DEG).
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import FORK_FOOT_NUTS_MASS_G, M4_NUT
from lib.yaw_coupler.layout import fork_foot_screw_points

CORNER_DEG = 90.0   # a corner here, from the pattern's +X: up the slot (the coupler's +Y), the flats across it

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = FORK_FOOT_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(fork_foot_screw_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "dropped into the slots in j1_coupler's disc from its top face, before j1_motor_leg slides on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The two plain nuts, each with a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.radians(CORNER_DEG), xy) for xy in fork_foot_screw_points())


@step
def fork_foot_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    fork_foot_nuts()   # build: writes the sibling fork_foot_nuts.step
