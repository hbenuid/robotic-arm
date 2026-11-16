"""coupler_plate_screws - purchased (COTS) part: the 4 M4 socket head cap screws (ISO 4762) that hold j1_coupler on
j1_coupler_plate.

In the arm (lib/mounts.py SPLIT_MOUNTS, hosted on j1_coupler#1): the heads in the counterbores in the ring's top face
(lib/yaw_coupler/layout.py HEAD_SEAT deep), the shanks down parallel to the base_yaw axis through j1_coupler and the
plate into the nuts pressed into the plate's underside (coupler_plate_nuts), the tips inside them. Length:
lib/yaw_coupler/params.py SplitParams.screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/base/test_fork_bolts.py locks its numbers. Frame: axis on Z (down
the base_yaw axis, into the plate), the heads' bearing face on z=0, the heads in -Z; the four at lib/yaw_coupler/layout.py
split_screw_points().
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import COUPLER_PLATE_SCREWS_MASS_G, M4_SHCS
from lib.yaw_coupler import DEFAULT as YAW_COUPLER
from lib.yaw_coupler.layout import split_screw_points

LENGTH = YAW_COUPLER.split.screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = COUPLER_PLATE_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(split_screw_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "j1_coupler onto j1_coupler_plate, down from the ring's top face into the nuts in the plate (coupler_plate_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M4_SHCS, LENGTH, xy) for xy in split_screw_points())


@step(out="coupler_plate_screws.step")
def coupler_plate_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    coupler_plate_screws()   # build: writes the sibling coupler_plate_screws.step
