"""fork_hub_screws - purchased (COTS) part: the 4 M4 socket head cap screws (ISO 4762) that hold the cycloidal drive's
output hub to j1_coupler's hub-side leg.

In the arm (lib/mounts.py FORK_MOUNTS, hosted on j1_coupler#1): the heads in the counterbores in the leg's outer face
(lib/yaw_coupler/layout.py HEAD_SEAT deep), the shanks along the drive's axis through the leg and the hub's arm-mount
holes into the nuts captive in the hub's flange (fork_hub_nuts), the tips flush with the nuts. Length:
lib/yaw_coupler/params.py ForkParams.hub_screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_fork_bolts.py locks its numbers. Frame: axis on Z (the
drive's axis, the shanks toward the hub), the heads' bearing face on z=0, the heads in -Z; the four at
lib/yaw_coupler/layout.py fork_hub_screw_points() - the hub's arm-mount pattern seen from the heads.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import CYCLOIDAL_HUB_BOLT_COUNT, FORK_HUB_SCREWS_MASS_G, M4_SHCS
from lib.yaw_coupler import DEFAULT as YAW_COUPLER
from lib.yaw_coupler.layout import fork_hub_screw_points

LENGTH = YAW_COUPLER.fork.hub_screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = FORK_HUB_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = CYCLOIDAL_HUB_BOLT_COUNT   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "the drive's output hub to j1_coupler's hub-side leg, into the nuts captive in the hub's flange (fork_hub_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M4_SHCS, LENGTH, xy) for xy in fork_hub_screw_points())


@step(out="fork_hub_screws.step")
def fork_hub_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    fork_hub_screws()   # build: writes the sibling fork_hub_screws.step
