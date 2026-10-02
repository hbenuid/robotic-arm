"""yaw_pulley_nuts - purchased (COTS) part: the base_yaw 120T's 4 M4 hex nuts (ISO 4032), in j1_coupler's hub.

The base_yaw 120T's screws run into them, in the hex pockets of the floor of the pocket over the coupler's hub.

In the arm (lib/mounts.py, hosted on yaw_pulley_screws): each nut on its pocket's floor (lib/yaw_coupler/params.py
HubParams.nut_depth: sunk flush with the floor of the pocket over the hub), all four with a corner along the coupler's
Z as the pockets are cut (lib/yaw_coupler/body.py); the pockets are nut_af across flats, a press for the nut
(docs/open_issues.md). They go in from above, through the pocket over the hub, before the drive is lowered into the
coupler's fork.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/base/test_yaw_pulley_bolts.py locks its numbers.
Frame: axis on Z, the nuts' bearing faces on z=0, standing in +Z; the four on the 90T's hole circle (lib/belts.py
pulley_90t_bolt_points()), each with a corner on the pattern's x = -y diagonal (CORNER_DEG): the coupler's Z where the
arm mounts them, the pattern turned with the pulley onto the stub's diagonal holes (HubParams.hole_deg).
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import M4_NUT, PULLEY_NUTS_MASS_G, pulley_90t_bolt_points
from lib.yaw_coupler import DEFAULT as YAW_COUPLER

CORNER_DEG = YAW_COUPLER.hub.hole_deg - 90.0   # a corner here, from the pattern's +X: the coupler's Z, the pulley's turn undone

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = PULLEY_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(pulley_90t_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "pressed into the hex pockets in the floor of the pocket over j1_coupler's hub, before the drive goes in"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, each with a corner at CORNER_DEG: the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.radians(CORNER_DEG), xy) for xy in pulley_90t_bolt_points())


@step
def yaw_pulley_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_pulley_nuts()   # build: writes the sibling yaw_pulley_nuts.step
