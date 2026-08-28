"""bearing_6003 - 6003-2RS deep-groove ball bearing 17x35x10 (simplified: a plain annulus, the drive repo's purchased-part model).

Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/bearing_6003.step is that
builder's export (kind "cots"). vendor/bearing_6003.step, when present, is a step.parts catalog model
re-oriented by VENDOR_TO_REF into the same frame: axis Z, standing on z=0. In the drive: x2, one per cycloidal disc on the eccentric-shaft lobes (stack z_disc1 / z_disc2).
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from build123d import Location  # noqa: E402  (import after shim)
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig  # noqa: E402
from lib.cycloidal.geom import cylinder  # noqa: E402
from lib.params import BEARING_6003_MASS_G  # noqa: E402
from parts.cycloidal._cots import hybrid  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BEARING_6003_MASS_G
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = Location()   # set after inspecting a step.parts model (see vendor/README.md)


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    b = cfg.bearings
    return cylinder(b.ecc_od / 2.0, b.ecc_width) - cylinder(b.ecc_bore / 2.0, b.ecc_width)


def gen_step():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
