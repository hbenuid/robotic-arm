"""fork_foot_screws - purchased (COTS) part: the 2 M4 socket head cap screws (ISO 4762) that hold j1_motor_leg's foot to
j1_coupler's disc.

In the arm (lib/mounts.py FORK_MOUNTS, hosted on j1_motor_leg#1): the heads in the counterbores in the foot
(ForkParams.motor_head_seat out from the face it butts on), the shanks along the drive's axis through the foot and the
disc's wall into the nuts in the slots in the disc (fork_foot_nuts), the tips past them. Length:
lib/yaw_coupler/params.py ForkParams.motor_screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_fork_bolts.py locks its numbers. Frame: axis on Z (along
the drive's axis, the shanks into the disc), the heads' bearing face on z=0, the heads in -Z; the two at
lib/yaw_coupler/layout.py fork_foot_screw_points() - motor_bolt_z either side of the drive's axis.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import FORK_FOOT_SCREWS_MASS_G, M4_SHCS
from lib.yaw_coupler import DEFAULT as YAW_COUPLER
from lib.yaw_coupler.layout import fork_foot_screw_points

LENGTH = YAW_COUPLER.fork.motor_screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = FORK_FOOT_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(fork_foot_screw_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "j1_motor_leg's foot onto j1_coupler's disc, into the nuts in the disc's slots (fork_foot_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The two plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M4_SHCS, LENGTH, xy) for xy in fork_foot_screw_points())


@step
def fork_foot_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    fork_foot_screws()   # build: writes the sibling fork_foot_screws.step
