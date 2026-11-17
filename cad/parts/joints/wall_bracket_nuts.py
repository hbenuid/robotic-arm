"""wall_bracket_nuts - purchased (COTS) part: the 3 M3 hex nuts (ISO 4032) in j2_wall_bracket's tongue, for the screws
that hold it on j2_link's web.

In the arm (lib/mounts.py SPLIT_MOUNTS, hosted on wall_bracket_screws): each nut in its hex pocket in the tongue's top
face (lib/forearm/params.py WallSplitParams.nut_fit), on the pocket's floor (lib/forearm/layout.py lap_nut_z), flush
with the top face, a corner along j2_link's x. They go in before the wrist motor: its face covers the third.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/forearm/test_wall_split.py locks its numbers. Frame:
wall_bracket_screws' - axis on Z, the nuts' bearing faces on z=0, standing in +Z; the three at
WallSplitParams.screw_points, a corner along +X.
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import DEFAULT as FOREARM
from lib.params import WALL_BRACKET_NUTS_MASS_G

SPLIT = FOREARM.split

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WALL_BRACKET_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SPLIT.nut.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(SPLIT.screw_points)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "in the hex pockets in j2_wall_bracket's tongue, before the wrist motor goes on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The three plain nuts, a corner along +X (the pockets'): the geometry until a vendor model exists."""
    return pattern(hex_nut(SPLIT.nut, 0.0, xy) for xy in SPLIT.screw_points)


@step(out="wall_bracket_nuts.step")
def wall_bracket_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wall_bracket_nuts()   # build: writes the sibling wall_bracket_nuts.step
