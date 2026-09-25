"""cycloidal_ring_pins - the 21 ring pins (4 x 35 mm h6 dowels) on the 108 mm circle, standing on z=0.

In the drive: at stack z_ring_pins (5.5) - 3.5 mm in the motor plate, 28 mm across the bore, 3.5 mm
into the ring gear body's bearing-zone wall. The disc lobes roll on them.
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_ring_pins.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_ring_pins.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, ring_pin_points
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import CYCLOIDAL_RING_PINS_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_RING_PINS_MASS_G
PURCHASE_SPEC = "{0.ring_pin_dia:g} x {0.ring_pin_length:g} mm h6 hardened ground dowel pin".format(DEFAULT_CONFIG.gear)
PURCHASE_QTY = DEFAULT_CONFIG.gear.num_ring_pins    # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "sold in packs - buy spares (25)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    g = cfg.gear
    return pattern(cylinder(g.ring_pin_radius, g.ring_pin_length, xy) for xy in ring_pin_points(cfg))


@step
def cycloidal_ring_pins():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    cycloidal_ring_pins()   # build: writes the sibling cycloidal_ring_pins.step
