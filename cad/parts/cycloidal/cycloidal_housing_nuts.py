"""cycloidal_housing_nuts - the M4 hex nuts (bolt_count) (7 AF x 3.2) on the bolt circle, standing on z=0, keyed like
their pockets (a corner radially outward, turned bolt_nut_turn_deg).

In the drive: at stack z_housing_nuts (56) in the ring gear body's output-face pockets. Solid
hexagons (no thread).
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_housing_nuts.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_housing_nuts.step would be
re-oriented by VENDOR_TO_REF into the same frame. The export is the port's 8-nut pattern (REFERENCE_BUILD); the model
builds DEFAULT_CONFIG's.
"""
import math
import pathlib

from cadgen import build123d as bd
from cadgen import step

from lib.cots import hybrid, pattern
from lib.cycloidal import DEFAULT_CONFIG, LEGACY_CONFIG, DriveConfig, compute_housing_bolt_angles, housing_bolt_points
from lib.datum import IDENTITY
from lib.geom import hex_prism
from lib.params import CYCLOIDAL_HOUSING_NUTS_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_HOUSING_NUTS_MASS_G
PURCHASE_SPEC = "M{0.bolt_dia:g} hex nut, {0.bolt_nut_af:g} AF x {0.bolt_nut_thickness:g} (ISO 4032)".format(DEFAULT_CONFIG.housing)
PURCHASE_QTY = DEFAULT_CONFIG.housing.bolt_count    # pieces per occurrence (the whole pattern)
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    h = cfg.housing
    turn = math.radians(h.bolt_nut_turn_deg)
    return pattern(bd.Pos(xy[0], xy[1], 0) * hex_prism(h.bolt_nut_af, angle + turn, h.bolt_nut_thickness)
                   for angle, xy in zip(compute_housing_bolt_angles(cfg), housing_bolt_points(cfg), strict=True))


def REFERENCE_BUILD():
    """The CadQuery port's pattern (LEGACY_CONFIG) - what reference/cycloidal/cycloidal_housing_nuts.step holds."""
    return _envelope(LEGACY_CONFIG)


@step
def cycloidal_housing_nuts():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    cycloidal_housing_nuts()   # build: writes the sibling cycloidal_housing_nuts.step
