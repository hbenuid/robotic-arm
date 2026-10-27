"""fork_sleeve_nuts - purchased (COTS) part: the 3 M4 hex nuts (ISO 4032) pressed into the lugs on the cycloidal drive's
motor plate's sleeve, for the screws that bolt j1_motor_leg's ring to it.

In the arm (lib/mounts.py FORK_MOUNTS, hosted on fork_sleeve_screws, in its frame): each nut on the floor of its pocket
in a lug (lib/yaw_coupler/layout.py fork_sleeve_nut_r: ShellParams.sleeve_nut_r to the nut's outer face), its flats
across the drive's axis, a corner along the tangent, as the pockets are cut (parts/cycloidal/cycloidal_motor_plate.py);
the pockets are ShellParams.sleeve_nut_af across flats, a press for the nut (docs/open_issues.md). They are pressed in
from inside the sleeve before the motor goes in.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/base/test_fork_bolts.py locks its numbers. Frame:
fork_sleeve_screws' - the drive's axis on Z, each nut's bearing face fork_sleeve_nut_r() out at its screw's angle,
standing in toward the axis (lib/yaw_coupler/body.py toward_axis), a corner along the tangent.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import CYCLOIDAL_SLEEVE_BOLT_COUNT, FORK_SLEEVE_NUTS_MASS_G, M4_NUT
from lib.yaw_coupler.body import toward_axis
from lib.yaw_coupler.layout import fork_sleeve_bolt_deg, fork_sleeve_nut_r

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = FORK_SLEEVE_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = CYCLOIDAL_SLEEVE_BOLT_COUNT   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "pressed into the lugs' pockets in the drive's motor plate's sleeve from inside, before the motor goes in"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The three plain nuts, each with a corner along the tangent: the geometry until a vendor model exists."""
    return pattern(toward_axis(hex_nut(M4_NUT, 0.0), deg, fork_sleeve_nut_r()) for deg in fork_sleeve_bolt_deg())


@step(out="fork_sleeve_nuts.step")
def fork_sleeve_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    fork_sleeve_nuts()   # build: writes the sibling fork_sleeve_nuts.step
