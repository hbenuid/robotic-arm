"""bearing_625 - 625-2RS deep-groove ball bearing 5x16x5 (simplified: a plain annulus, the drive repo's purchased-part model).

Ported from cycloidal_drive@2f1f67d src/purchased_parts.py; reference/bearing_625.step is that
builder's export (kind "cots"). vendor/bearing_625.step, when present, is a step.parts catalog model
re-oriented by VENDOR_TO_REF into the same frame: axis Z, standing on z=0. In the drive: x1 in the output hub's inner-face pocket (z 37), on the shaft support dowel.
"""
import pathlib

from cadgen import step

from lib.cycloidal import DEFAULT_CONFIG, DriveConfig
from lib.cycloidal.geom import cylinder
from lib.datum import IDENTITY
from lib.params import BEARING_625_MASS_G
from parts.cycloidal._cots import hybrid

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BEARING_625_MASS_G
PURCHASE_SPEC = "625-2RS miniature ball bearing, {0.inp_bore:g} x {0.inp_od:g} x {0.inp_width:g}".format(DEFAULT_CONFIG.bearings)
PURCHASE_QTY = 1    # pieces per occurrence
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY   # set after inspecting a step.parts model (see vendor/README.md)


def _envelope(cfg: DriveConfig = DEFAULT_CONFIG):
    b = cfg.bearings
    return cylinder(b.inp_od / 2.0, b.inp_width) - cylinder(b.inp_bore / 2.0, b.inp_width)


@step
def bearing_625():
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    bearing_625()   # build: writes the sibling bearing_625.step
