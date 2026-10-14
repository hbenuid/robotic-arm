"""gt2_idler_20t - purchased (COTS) part: GT2 20T toothless (smooth) idler, 5 mm bore, for the 6 mm belt - two flanges on
a smooth belt seat, a bearing inside on the bore (not modelled: the envelope is one solid).

No vendor model (step.parts' 5 mm-bore idlers are one generic model, not this one - vendor/README.md) and no
SolidWorks export: a NATIVE COTS part (lib/reference.py NATIVE_COTS) - its envelope IS the geometry, built from the
seller's drawing, and its reference is that envelope (reference/native/gt2_idler_20t.step,
tools/reference/import_native.py). Frame: axis on Z, standing on z=0.
Dimensions: lib/belts.py GT2_IDLER_*. In the arm: x3 - the roll belt's tensioner, in forearm_roll_drive
(lib/forearm/params.py RollIdlerParams) - on the riser on the roll frame's front face, on an M5 SHCS down from its top
(roll_idler_screw) into a nut under the face (roll_idler_nut), slid along N to tension the belt; and the elbow belt's
(gt2_idler_20t#1, lib/upper_arm/params.py ElbowIdlerParams) - under j1_link's slab below its printed spacer
(elbow_idler_spacer), on an M5 SHCS up from under it (elbow_idler_screw) into a nut in a slot down from the slab's top
face (elbow_idler_nut), slid along the link's Z; and the base_yaw belt's (gt2_idler_20t#2, lib/base/params.py
YawIdlerParams) - on base_motor_mount's plate on its printed spacer (yaw_idler_spacer), on an M5 SHCS down from its top
(yaw_idler_screw) into a nut between two rails under the plate (yaw_idler_nut), slid across the belt. lib/forearm/params.py
IdlerParams keeps the wrist mount it had (an M5 SHCS up from under it, wrist_idler_screw, through a rib and slot under
j2_link's web into a nut in the web's top, wrist_idler_nut), unplaced: the wrist belt's tensioner is still open
(docs/open_issues.md).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.geom import cylinder, through
from lib.params import (
    GT2_BELT_W,
    GT2_IDLER_BORE,
    GT2_IDLER_CHANNEL_W,
    GT2_IDLER_FLANGE_DIA,
    GT2_IDLER_SEAT_DIA,
    GT2_IDLER_WIDTH,
)

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 5.0    # [ESTIMATE] aluminium body + its bearing; weigh one
PURCHASE_SPEC = (f"GT2 20T toothless idler, {GT2_IDLER_BORE:g} mm bore, {GT2_BELT_W:g} mm belt, "
                 f"{GT2_IDLER_FLANGE_DIA:g} mm flanges x {GT2_IDLER_WIDTH:g}")
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = "sold in 5-packs; a bearing inside"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The flanges, the belt seat between them and the bore: the geometry until a vendor model exists."""
    flange_t = (GT2_IDLER_WIDTH - GT2_IDLER_CHANNEL_W) / 2.0
    body = (cylinder(GT2_IDLER_FLANGE_DIA / 2.0, flange_t)
            + cylinder(GT2_IDLER_SEAT_DIA / 2.0, GT2_IDLER_CHANNEL_W, z0=flange_t)
            + cylinder(GT2_IDLER_FLANGE_DIA / 2.0, flange_t, z0=GT2_IDLER_WIDTH - flange_t))
    return body - through(GT2_IDLER_BORE / 2.0, GT2_IDLER_WIDTH)


@step
def gt2_idler_20t():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    gt2_idler_20t()   # build: writes the sibling gt2_idler_20t.step
