"""wrist_roll_clamp_nuts - purchased (COTS) part: the wrist roll rotor clamp's 4 M3 hex nuts (ISO 4032), in the shaft.

The rotor clamp's screws (wrist_roll_clamp_screws) run into them from the pulley's face. Each sits in its pocket in the
shaft's flange (lib/wrist/layout.py clamp_nut_pocket: a slot from the cable bore, the nut's across flats wide - its
flats hold it from turning -, open to the shaft's rear face), its bearing face on the pocket's end, 2 pitches short of
the screw's tip. Push them in from the shaft's rear face (in wrist_link's bay, the back panel off) before the shaft goes
in.

In the arm: x1, in wrist_roll_drive (its row in assemblies/wrist_roll_drive.py, the rotor - wrist_roll_link).

No catalog model, no SolidWorks export and NO reference file: a pattern part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py hex_nut()) and tests/wrist/test_roll_screws.py locks its
numbers. Frame: axis on Z, each nut's far face on z=0 (its bearing face at z = h), a corner along its radius; the four on
the clamp's circle (lib/wrist/layout.py clamp_points(), as the module has them: the row is a translation).
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import WRIST_ROLL_CLAMP_NUTS_MASS_G
from lib.wrist import DEFAULT as WRIST
from lib.wrist.layout import clamp_points

NUT = WRIST.roll.nut
POINTS = tuple(clamp_points(WRIST))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_CLAMP_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "in the wrist roll shaft's flange, for the rotor clamp: pushed into the pockets from the shaft's rear face"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, a corner along each one's radius: the geometry until a vendor model exists."""
    return pattern(hex_nut(NUT, math.atan2(y, x), (x, y)) for x, y in POINTS)


@step(out="wrist_roll_clamp_nuts.step")
def wrist_roll_clamp_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_clamp_nuts()   # build: writes the sibling wrist_roll_clamp_nuts.step
