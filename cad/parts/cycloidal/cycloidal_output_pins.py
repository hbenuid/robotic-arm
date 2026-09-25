"""cycloidal_output_pins - the 4 output pins (4 x 45 mm h6 dowels) on the 60 mm circle, standing on z=0.

In the drive: at stack z_output_pins (11), captured between the hub's blind holes (1 mm ceiling)
and the motor plate; they ride in the discs' 7.4 mm holes and carry the output torque.
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_output_pins.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_output_pins.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, output_pin_points
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import CYCLOIDAL_OUTPUT_PINS_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_OUTPUT_PINS_MASS_G
PURCHASE_SPEC = "{0.output_pin_dia:g} x {0.output_pin_length:g} mm h6 hardened ground dowel pin".format(DEFAULT_CONFIG.disc)
PURCHASE_QTY = DEFAULT_CONFIG.disc.output_pin_count    # pieces per occurrence (the whole pattern)
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    d = cfg.disc
    return pattern(cylinder(d.output_pin_dia / 2.0, d.output_pin_length, xy) for xy in output_pin_points(cfg))


@step
def cycloidal_output_pins():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    cycloidal_output_pins()   # build: writes the sibling cycloidal_output_pins.step
