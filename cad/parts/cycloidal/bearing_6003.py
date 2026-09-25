"""bearing_6003 - 6003-2RS deep-groove ball bearing 17x35x10 (simplified: a plain annulus, the drive repo's purchased-part model).

Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/bearing_6003.step is that
builder's export (kind "cots"). vendor/bearing_6003.step, when present, is a step.parts catalog model
re-oriented by VENDOR_TO_REF into the same frame: axis Z, standing on z=0. In the drive: x2, one per cycloidal disc on the eccentric-shaft lobes (stack z_disc1 / z_disc2).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import BEARING_6003_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BEARING_6003_MASS_G
PURCHASE_SPEC = "6003-2RS deep-groove ball bearing, {0.ecc_bore:g} x {0.ecc_od:g} x {0.ecc_width:g}".format(DEFAULT_CONFIG.bearings)
PURCHASE_QTY = 1    # pieces per occurrence (the drive places it twice)
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY   # set after inspecting a step.parts model (see vendor/README.md)


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    b = cfg.bearings
    return cylinder(b.ecc_od / 2.0, b.ecc_width) - cylinder(b.ecc_bore / 2.0, b.ecc_width)


@step
def bearing_6003():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    bearing_6003()   # build: writes the sibling bearing_6003.step
