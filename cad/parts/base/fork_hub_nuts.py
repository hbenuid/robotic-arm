"""fork_hub_nuts - purchased (COTS) part: the 4 M4 hex nuts (ISO 4032) captive in the cycloidal drive's output hub, for
the screws that hold it to j1_coupler's hub-side leg.

In the arm (lib/mounts.py FORK_MOUNTS, hosted on fork_hub_screws): each nut on the floor of its pocket in the hub's
flange (lib/cycloidal/params.py ShellParams.hub_nut_depth in from its inner face: lib/yaw_coupler/layout.py
fork_hub_nut_z), a corner toward the hub's axis as the pockets are cut (lib/cycloidal/housing.py hex_pocket); the
pockets are HousingParams.bolt_nut_pocket_af across flats, a slip fit. They drop into the flange's inner face before the
hub is pressed through its 6814.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/base/test_fork_bolts.py locks its numbers. Frame:
fork_hub_screws' - axis on Z, the nuts' bearing faces on z=0, standing in +Z; each at a point of
lib/yaw_coupler/layout.py fork_hub_screw_points() with a corner along its radius.
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import CYCLOIDAL_HUB_BOLT_COUNT, FORK_HUB_NUTS_MASS_G, M4_NUT
from lib.yaw_coupler.layout import fork_hub_screw_points

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = FORK_HUB_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = CYCLOIDAL_HUB_BOLT_COUNT   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "captive in the drive's output hub's flange, before the hub is pressed through its 6814"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, each with a corner along its radius: the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.atan2(y, x), (x, y)) for x, y in fork_hub_screw_points())


@step
def fork_hub_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    fork_hub_nuts()   # build: writes the sibling fork_hub_nuts.step
