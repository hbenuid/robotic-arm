"""wrist_pulley_nuts - purchased (COTS) part: the wrist 90T's 4 M4 hex nuts (ISO 4032), in j3_coupler's flange.

The wrist 90T's screws run into them, in the hex pockets of the coupler flange's underside.

In the arm (lib/mounts.py, hosted on wrist_pulley_screws): each nut on its pocket's floor (lib/coupler/params.py
CouplerParams.nut_depth, the nut standing proud of the flange by the rest of its height into wrist_link's central
hole), all four with a corner along the coupler's Z as the pockets are cut (lib/coupler/body.py); the pockets are
nut_af across flats, a press for the nut (docs/open_issues.md). They go in before wrist_link covers the flange.

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like the drive's cycloidal_housing_nuts - its envelope IS the geometry
(lib/fasteners.py hex_nut(): no thread, the bore at the nominal diameter) and its reference is that envelope
(reference/native/wrist_pulley_nuts.step, tools/reference/import_native.py). Frame: axis on Z, the nuts' bearing faces
on z=0, standing in +Z; the four on the 90T's hole circle (lib/belts.py pulley_90t_bolt_points()), each with a corner
along +/-Y (the coupler's Z where the arm mounts them).
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import M4_NUT, PULLEY_NUTS_MASS_G, pulley_90t_bolt_points

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = PULLEY_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(pulley_90t_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "pressed into the hex pockets in j3_coupler's flange underside, before wrist_link goes on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, each with a corner along +/-Y: the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.pi / 2.0, xy) for xy in pulley_90t_bolt_points())


@step
def wrist_pulley_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_pulley_nuts()   # build: writes the sibling wrist_pulley_nuts.step
