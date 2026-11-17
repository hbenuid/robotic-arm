"""wall_bracket_screws - purchased (COTS) part: the 3 M3 socket head cap screws (ISO 4762) that hold j2_wall_bracket's
tongue on j2_link's web.

In the arm (lib/mounts.py SPLIT_MOUNTS, hosted on j2_link#1): the heads flush in the counterbores in the web's
underside (lib/forearm/params.py WallSplitParams.head_dia, head_h deep - lib/forearm/layout.py lap_screw_head_z), the
shanks up through the web's end and the tongue into the nuts in the tongue's top face (wall_bracket_nuts), the tips
flush with it. Length: WallSplitParams.screw_len. The wrist motor's two front screws (wrist_motor_screws) pass
through the lap beside them.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_wall_split.py locks its numbers. Frame: axis on Z (up
j2_link's +Z), the heads' bearing face on z=0, the heads in -Z; the three at WallSplitParams.screw_points, in j2_link's
(x, y).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import DEFAULT as FOREARM
from lib.params import WALL_BRACKET_SCREWS_MASS_G

SPLIT = FOREARM.split
LENGTH = SPLIT.screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WALL_BRACKET_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SPLIT.screw.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(SPLIT.screw_points)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "j2_wall_bracket onto j2_link, up from the web's underside into the nuts in the bracket's tongue (wall_bracket_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The three plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SPLIT.screw, LENGTH, xy) for xy in SPLIT.screw_points)


@step(out="wall_bracket_screws.step")
def wall_bracket_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wall_bracket_screws()   # build: writes the sibling wall_bracket_screws.step
