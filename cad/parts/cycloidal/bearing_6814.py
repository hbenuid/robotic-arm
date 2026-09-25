"""bearing_6814 - 6814-2RS deep-groove ball bearing 70x90x10 (simplified: a plain annulus, the drive repo's purchased-part model).

Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/bearing_6814.step is that
builder's export (kind "cots"). vendor/bearing_6814.step, when present, is a step.parts catalog model
re-oriented by VENDOR_TO_REF into the same frame: axis Z, standing on z=0. In the drive: x2 stacked in the ring gear body seat (z 37 / 47), inner races on the output hub.
"""
import pathlib

from cadgen import step

from lib.cycloidal import DEFAULT_CONFIG, DriveConfig
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import BEARING_6814_MASS_G
from parts.cycloidal._cots import hybrid

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BEARING_6814_MASS_G
PURCHASE_SPEC = "6814-2RS (61814) thin-section ball bearing, {0.out_bore:g} x {0.out_od:g} x {0.out_width:g}".format(DEFAULT_CONFIG.bearings)
PURCHASE_QTY = 1    # pieces per occurrence (the drive places it twice)
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY   # set after inspecting a step.parts model (see vendor/README.md)


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    b = cfg.bearings
    return cylinder(b.out_od / 2.0, b.out_width) - cylinder(b.out_bore / 2.0, b.out_width)


@step
def bearing_6814():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    bearing_6814()   # build: writes the sibling bearing_6814.step
