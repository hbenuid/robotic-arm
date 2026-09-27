"""washer_as6590 - purchased (COTS) part: INA AS 6590 thrust washer, 65 x 90 x 1 - the raceways of the base_yaw thrust
bearing (bearing_axk6590), one each side of it: the lower on the floor of the base's groove, the upper under
j1_coupler's seat (lib/mounts.py THRUST_MOUNTS; lib/bearings.py the stack).

No vendor model and no SolidWorks export: a NATIVE COTS part (lib/reference.py NATIVE_COTS) - its envelope IS the
geometry, and its reference is that envelope (reference/native/washer_as6590.step, tools/reference/import_native.py).
Frame: axis on Z, standing on z=0. Dimensions: lib/bearings.py. In the arm: x2.
"""
import pathlib

from cadgen import step

from lib.bearings import THRUST_BORE, THRUST_OD, THRUST_WASHER_MASS_G, THRUST_WASHER_WIDTH
from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.geom import cylinder

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = THRUST_WASHER_MASS_G   # [DATASHEET] see lib/bearings.py
PURCHASE_SPEC = f"INA AS {THRUST_BORE:g}{THRUST_OD:g} thrust washer, {THRUST_BORE:g} x {THRUST_OD:g} x {THRUST_WASHER_WIDTH:g}"
PURCHASE_QTY = 1    # pieces per occurrence (the arm places two: one each side of the cage)
PURCHASE_NOTE = "hardened: the needles' raceway on both sides (the base and j1_coupler are PETG)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The annulus (OD, bore, width): the geometry until a vendor model exists."""
    return cylinder(THRUST_OD / 2.0, THRUST_WASHER_WIDTH) - cylinder(THRUST_BORE / 2.0, THRUST_WASHER_WIDTH)


@step
def washer_as6590():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    washer_as6590()   # build: writes the sibling washer_as6590.step
