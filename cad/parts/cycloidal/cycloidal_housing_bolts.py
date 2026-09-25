"""cycloidal_housing_bolts - the 8 M4 x 55 socket-head screws clamping the housing (head top at z=0, shank in +Z).

In the drive: at stack z_housing_bolts (0.5) - heads in the motor plate's counterbores, shanks
through both housing parts into the captive nuts. Plain cylinders (no thread, no socket).
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_housing_bolts.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_housing_bolts.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, housing_bolt_points
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import CYCLOIDAL_HOUSING_BOLTS_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_HOUSING_BOLTS_MASS_G
PURCHASE_SPEC = "M{0.bolt_dia:g} x {0.bolt_length:g} socket head cap screw (ISO 4762)".format(DEFAULT_CONFIG.housing)
PURCHASE_QTY = DEFAULT_CONFIG.housing.bolt_count    # pieces per occurrence (the whole pattern)
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    h = cfg.housing
    return pattern(cylinder(h.bolt_head_dia / 2.0, h.bolt_head_height, xy) + cylinder(h.bolt_dia / 2.0, h.bolt_length, xy, z0=h.bolt_head_height)
                   for xy in housing_bolt_points(cfg))


@step
def cycloidal_housing_bolts():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    cycloidal_housing_bolts()   # build: writes the sibling cycloidal_housing_bolts.step
