"""cycloidal_shaft_support_pin - the 5 x 20 mm h6 dowel supporting the eccentric shaft's output end, standing on z=0.

In the drive: at stack z_support_pin (24) - 11 mm pressed into the shaft, 9 mm proud through the
2 mm gap into the 625 bearing in the output hub.
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_shaft_support_pin.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_shaft_support_pin.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from build123d import Location, Pos  # noqa: E402  (import after shim)
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, stack_positions  # noqa: E402
from lib.cycloidal.geom import cylinder  # noqa: E402
from lib.cycloidal.housing import hex_prism  # noqa: E402,F401
from lib.params import CYCLOIDAL_SUPPORT_PIN_MASS_G  # noqa: E402
from parts.cycloidal._cots import hybrid, pattern  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_SUPPORT_PIN_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    s = cfg.shaft
    return cylinder(s.support_pin_dia / 2.0, s.support_pin_length)


def gen_step():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
