"""yaw_pulley_screws - purchased (COTS) part: the base_yaw 90T's 4 M4 socket head cap screws (ISO 4762).

They clamp the pulley onto j1_coupler's stub (gt2_pulley_90t#5 -> j1_coupler#1), which holds the coupler down on its
thrust bearing: the pulley's ring sits under the lower base bearing's inner ring.

In the arm (lib/mounts.py, hosted on the base_yaw 90T): the heads on the pulley's outer face (it faces down, inside
the base), the shanks up through its M4 clearance holes and the coupler's stub into the nuts in the hex pockets of the
floor of the pocket over its hub (yaw_pulley_nuts); the tips end past the nuts in that pocket, under the drive's
housing. Length: lib/yaw_coupler/params.py HubParams.pulley_screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: an ENVELOPE COTS pattern part (lib/reference.py ENVELOPE_COTS) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_yaw_pulley_bolts.py locks its numbers. Frame: axis on Z,
the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the 90T's hole circle (lib/belts.py
pulley_90t_bolt_points()).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import M4_SHCS, YAW_PULLEY_SCREWS_MASS_G, pulley_90t_bolt_points
from lib.yaw_coupler import DEFAULT as YAW_COUPLER

LENGTH = YAW_COUPLER.hub.pulley_screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = YAW_PULLEY_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(pulley_90t_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "the base_yaw 90T onto j1_coupler, into the nuts in the floor of the pocket over its hub (yaw_pulley_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M4_SHCS, LENGTH, xy) for xy in pulley_90t_bolt_points())


@step
def yaw_pulley_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    yaw_pulley_screws()   # build: writes the sibling yaw_pulley_screws.step
