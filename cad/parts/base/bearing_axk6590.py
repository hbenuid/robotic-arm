"""bearing_axk6590 - purchased (COTS) part: INA AXK 6590 needle roller and cage thrust assembly, 65 x 90 x 3 - the
base_yaw thrust bearing, between two washer_as6590 in the base's groove round its seat ring, under j1_coupler
(lib/mounts.py THRUST_MOUNTS places them; lib/bearings.py the stack).

No vendor model and no SolidWorks export: a NATIVE COTS part (lib/reference.py NATIVE_COTS) - its envelope IS the
geometry, and its reference is that envelope (reference/native/bearing_axk6590.step, tools/reference/import_native.py).
Frame: axis on Z, standing on z=0. Dimensions: lib/bearings.py. In the arm: x1.
"""
import pathlib

from cadgen import step

from lib.bearings import THRUST_BORE, THRUST_CAGE_MASS_G, THRUST_CAGE_WIDTH, THRUST_OD
from lib.cots import hybrid
from lib.datum import IDENTITY
from lib.geom import cylinder

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = THRUST_CAGE_MASS_G   # [DATASHEET] see lib/bearings.py
PURCHASE_SPEC = (f"INA AXK {THRUST_BORE:g}{THRUST_OD:g} needle roller and cage thrust assembly, "
                 f"{THRUST_BORE:g} x {THRUST_OD:g} x {THRUST_CAGE_WIDTH:g}")
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = "any brand of the DIN 5405-2 size (AXK 6590 / NTA-type 65 x 90); runs on two AS 6590 washers (washer_as6590)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The annulus (OD, bore, the needles' diameter): the geometry until a vendor model exists."""
    return cylinder(THRUST_OD / 2.0, THRUST_CAGE_WIDTH) - cylinder(THRUST_BORE / 2.0, THRUST_CAGE_WIDTH)


@step
def bearing_axk6590():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    bearing_axk6590()   # build: writes the sibling bearing_axk6590.step
