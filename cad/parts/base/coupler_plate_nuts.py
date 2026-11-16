"""coupler_plate_nuts - purchased (COTS) part: the 4 M4 hex nuts (ISO 4032) in j1_coupler_plate, for the screws that hold
j1_coupler on it.

In the arm (lib/mounts.py SPLIT_MOUNTS, hosted on coupler_plate_screws): each nut pressed up into its hex pocket in the
plate's underside (the recess's ceiling; lib/yaw_coupler/params.py SplitParams.nut_af), against the pocket's floor
(lib/yaw_coupler/layout.py split_nut_z), flush with the underside, a corner along the coupler's Z. They go in before
j1_coupler is bolted on, the plate off the base.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/base/test_fork_bolts.py locks its numbers. Frame:
coupler_plate_screws' - axis on Z, the nuts' bearing faces on z=0, standing in +Z; the four at lib/yaw_coupler/layout.py
split_screw_points(), a corner along +Y (CORNER_DEG).
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import COUPLER_PLATE_NUTS_MASS_G, M4_NUT
from lib.yaw_coupler.layout import split_screw_points

CORNER_DEG = 90.0   # a corner here, from the pattern's +X: along the coupler's Z (the pockets', SplitParams.nut_af)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = COUPLER_PLATE_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(split_screw_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "pressed up into the hex pockets in j1_coupler_plate's underside, before j1_coupler is bolted on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, each with a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.radians(CORNER_DEG), xy) for xy in split_screw_points())


@step(out="coupler_plate_nuts.step")
def coupler_plate_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    coupler_plate_nuts()   # build: writes the sibling coupler_plate_nuts.step
