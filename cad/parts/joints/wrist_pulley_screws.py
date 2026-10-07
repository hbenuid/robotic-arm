"""wrist_pulley_screws - purchased (COTS) part: the wrist 90T's 4 M4 socket head cap screws (ISO 4762).

They clamp the pulley onto j3_coupler's stub (gt2_pulley_90t#4 -> j3_coupler#2).

In the arm (lib/mounts.py, hosted on the wrist 90T): the heads in the counterbores in the pulley's outer face
(lib/belts.py GT2_PULLEY_90T_HEAD_SEAT), the shanks down through its M4 clearance holes and the coupler's stub into the nuts in the hex pockets of its flange's underside
(wrist_pulley_nuts); the tips end past the nuts inside wrist_link's central hole. Length: lib/coupler/params.py
CouplerParams.pulley_screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like the drive's cycloidal_housing_bolts - its envelope IS the geometry
(lib/fasteners.py shcs(): no thread, the head's hex socket) and its reference is that envelope
(reference/native/wrist_pulley_screws.step, tools/reference/import_native.py). Frame: axis on Z, the heads' bearing
face on z=0 (the heads in -Z, on the counterbores' floors), the shanks in +Z; the four on the 90T's hole circle (lib/belts.py
pulley_90t_bolt_points()).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.coupler import DEFAULT as COUPLER
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import M4_SHCS, WRIST_PULLEY_SCREWS_MASS_G, pulley_90t_bolt_points

LENGTH = COUPLER.pulley_screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_PULLEY_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(pulley_90t_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "the wrist 90T onto j3_coupler, into the nuts in its flange's pockets (wrist_pulley_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M4_SHCS, LENGTH, xy) for xy in pulley_90t_bolt_points())


@step
def wrist_pulley_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_pulley_screws()   # build: writes the sibling wrist_pulley_screws.step
