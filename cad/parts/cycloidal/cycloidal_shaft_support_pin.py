"""cycloidal_shaft_support_pin - the 5 x 20 mm h6 dowel supporting the eccentric shaft's output end, standing on z=0.

In the drive: at stack z_support_pin (24) - 11 mm pressed into the shaft, 9 mm proud through the
2 mm gap into the 625 bearing in the output hub.
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_shaft_support_pin.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_shaft_support_pin.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
import pathlib

from build123d import Location, Pos
from cadgen import step
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, stack_positions
from lib.cycloidal.geom import cylinder
from lib.params import CYCLOIDAL_SUPPORT_PIN_MASS_G
from parts.cycloidal._cots import hybrid, pattern

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_SUPPORT_PIN_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    s = cfg.shaft
    return cylinder(s.support_pin_dia / 2.0, s.support_pin_length)


@step
def cycloidal_shaft_support_pin():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    cycloidal_shaft_support_pin()   # build: writes the sibling cycloidal_shaft_support_pin.step (preview: ./cadtool show parts/cycloidal/cycloidal_shaft_support_pin.py)
