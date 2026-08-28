"""cycloidal_housing_bolts - the 8 M4 x 55 socket-head screws clamping the housing (head top at z=0, shank in +Z).

In the drive: at stack z_housing_bolts (0.5) - heads in the motor plate's counterbores, shanks
through both housing parts into the captive nuts. Plain cylinders (no thread, no socket).
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_housing_bolts.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_housing_bolts.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Location, Pos  # noqa: E402  (import after shim)
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, housing_bolt_points  # noqa: E402
from lib.cycloidal.geom import cylinder  # noqa: E402
from lib.cycloidal.housing import hex_prism  # noqa: E402,F401
from lib.params import CYCLOIDAL_HOUSING_BOLTS_MASS_G  # noqa: E402
from parts._cycloidal_cots import hybrid, pattern  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_HOUSING_BOLTS_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parent.parent / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    h = cfg.housing
    return pattern(cylinder(h.bolt_head_dia / 2.0, h.bolt_head_height, xy) + cylinder(h.bolt_dia / 2.0, h.bolt_length, xy, z0=h.bolt_head_height)
                   for xy in housing_bolt_points(cfg))


def gen_step():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
