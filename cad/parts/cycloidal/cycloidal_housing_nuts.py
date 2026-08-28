"""cycloidal_housing_nuts - the 8 M4 hex nuts (7 AF x 3.2) on the 125 mm bolt circle, standing on z=0, keyed radially.

In the drive: at stack z_housing_nuts (56) in the ring gear body's output-face pockets. Solid
hexagons (no thread).
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_housing_nuts.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_housing_nuts.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from build123d import Location, Pos  # noqa: E402  (import after shim)
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, compute_housing_bolt_angles, housing_bolt_points  # noqa: E402
from lib.cycloidal.geom import cylinder  # noqa: E402
from lib.cycloidal.housing import hex_prism  # noqa: E402,F401
from lib.params import CYCLOIDAL_HOUSING_NUTS_MASS_G  # noqa: E402
from parts.cycloidal._cots import hybrid, pattern  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_HOUSING_NUTS_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    h = cfg.housing
    return pattern(Pos(xy[0], xy[1], 0) * hex_prism(h.bolt_nut_af, angle, h.bolt_nut_thickness)
                   for angle, xy in zip(compute_housing_bolt_angles(cfg), housing_bolt_points(cfg)))


def gen_step():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
