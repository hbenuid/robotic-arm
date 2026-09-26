"""bearing_6806 - purchased (COTS) part: 6806-2RS (61806) thin-section deep-groove ball bearing, 30 x 42 x 7 - the pair
at each belt joint (base_yaw, elbow_pitch, wrist_pitch), one each side of the lip in the housing's bore (lib/mounts.py
places them: the coupler's stub in the upper one, the 90T pulley's hub in the lower one).

No vendor model (step.parts has no bearing above a 20 mm bore, vendor/README.md) and no SolidWorks export: a NATIVE
COTS part (lib/reference.py NATIVE_COTS) - its envelope IS the geometry, and its reference is that envelope
(reference/native/bearing_6806.step, tools/reference/import_native.py). Frame: axis on Z, standing on z=0.
Dimensions: lib/bearings.py. In the arm: x6.
"""
import pathlib

from cadgen import step

from lib.bearings import BEARING_6806_BORE, BEARING_6806_MASS_G, BEARING_6806_OD, BEARING_6806_WIDTH
from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.geom import cylinder

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BEARING_6806_MASS_G   # [ESTIMATE] see lib/bearings.py
PURCHASE_SPEC = f"6806-2RS (61806) thin-section deep-groove ball bearing, {BEARING_6806_BORE:g} x {BEARING_6806_OD:g} x {BEARING_6806_WIDTH:g}"
PURCHASE_QTY = 1    # pieces per occurrence (the arm places six: two per belt joint)
PURCHASE_NOTE = "any brand; sealed (2RS) - the belt side is open to the room"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The annulus (outer race OD, bore, width): the geometry until a vendor model exists."""
    return cylinder(BEARING_6806_OD / 2.0, BEARING_6806_WIDTH) - cylinder(BEARING_6806_BORE / 2.0, BEARING_6806_WIDTH)


@step
def bearing_6806():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    bearing_6806()   # build: writes the sibling bearing_6806.step
