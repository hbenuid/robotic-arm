"""fork_sleeve_screws - purchased (COTS) part: the 3 M4 socket head cap screws (ISO 4762) that bolt j1_motor_leg's ring to
the cycloidal drive's motor plate's sleeve - nothing else stops the carrier's motor end turning.

In the arm (lib/mounts.py FORK_MOUNTS, hosted on j1_motor_leg#1): radially through the ring's middle at the drive's
ShellParams.sleeve_bolt_deg (the motor's flats: up and both sides, none down where the leg's post stands), the heads on
the ring's od (no seat: ForkParams.plate_r), the shanks through the ring's slotted holes and the sleeve into the nuts
pressed into its lugs (fork_sleeve_nuts), the tips flush with the nuts. Length: lib/yaw_coupler/params.py
ForkParams.sleeve_screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_fork_bolts.py locks its numbers. Frame: the drive's -
axis on Z, the three in the z = 0 plane at the sleeve_bolt_deg from +X toward +Y, each head's bearing face plate_r
out, the shank pointing in at the axis (lib/yaw_coupler/body.py toward_axis).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import CYCLOIDAL_SLEEVE_BOLT_COUNT, FORK_SLEEVE_SCREWS_MASS_G, M4_SHCS
from lib.yaw_coupler import DEFAULT as YAW_COUPLER
from lib.yaw_coupler.body import toward_axis
from lib.yaw_coupler.layout import fork_sleeve_bolt_deg

LENGTH = YAW_COUPLER.fork.sleeve_screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = FORK_SLEEVE_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = CYCLOIDAL_SLEEVE_BOLT_COUNT   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("radially through j1_motor_leg's ring into the nuts in the drive's motor plate's sleeve (fork_sleeve_nuts); "
                 "after the foot's screws, snug - they pull the sleeve across the ring's 0.2 mm")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The three plain screws: the geometry until a vendor model exists."""
    return pattern(toward_axis(shcs(M4_SHCS, LENGTH), deg, YAW_COUPLER.fork.plate_r) for deg in fork_sleeve_bolt_deg())


@step(out="fork_sleeve_screws.step")
def fork_sleeve_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    fork_sleeve_screws()   # build: writes the sibling fork_sleeve_screws.step
