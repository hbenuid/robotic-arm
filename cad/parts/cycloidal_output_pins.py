"""cycloidal_output_pins - the 4 output pins (4 x 45 mm h6 dowels) on the 60 mm circle, standing on z=0.

In the drive: at stack z_output_pins (11), captured between the hub's blind holes (1 mm ceiling)
and the motor plate; they ride in the discs' 7.4 mm holes and carry the output torque.
Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/cycloidal_output_pins.step is that builder's
export (kind "cots"). No catalog model (the envelope is the geometry); a vendor/cycloidal_output_pins.step would be
re-oriented by VENDOR_TO_REF into the same frame.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Location, Pos  # noqa: E402  (import after shim)
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig, output_pin_points  # noqa: E402
from lib.cycloidal.geom import cylinder  # noqa: E402
from lib.cycloidal.housing import hex_prism  # noqa: E402,F401
from lib.params import CYCLOIDAL_OUTPUT_PINS_MASS_G  # noqa: E402
from parts._cycloidal_cots import hybrid, pattern  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = CYCLOIDAL_OUTPUT_PINS_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parent.parent / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    d = cfg.disc
    return pattern(cylinder(d.output_pin_dia / 2.0, d.output_pin_length, xy) for xy in output_pin_points(cfg))


def gen_step():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
