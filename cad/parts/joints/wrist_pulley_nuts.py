"""wrist_pulley_nuts - purchased (COTS) part: the wrist 90T's 4 M4 hex nuts (ISO 4032), in wrist_link's plate.

The wrist 90T's screws run into them through the wrist flange, in hex channels from the seats in wrist_link's plate down
through it.

In the arm (lib/mounts.py, hosted on wrist_pulley_screws): each nut on its channel's seat (lib/wrist/params.py
FlangeParams.nut_seat_z - 1 mm under the plate's coupler face, past the wrist flange), all
four turned with the 90T (FlangeParams.pulley_bolt_deg) as the channels are cut (+ nut_corner_deg); the channels are nut_af
across flats, a press for the nut (docs/open_issues.md). They drop in from the pancake's pocket before the pancake.

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like the drive's cycloidal_housing_nuts - its envelope IS the geometry
(lib/fasteners.py hex_nut(): no thread, the bore at the nominal diameter) and its reference is that envelope
(reference/native/wrist_pulley_nuts.step, tools/reference/import_native.py). Frame: axis on Z, the nuts' bearing faces
on z=0, standing in +Z; the four on the 90T's hole circle (lib/belts.py pulley_90t_bolt_points()), each with a corner
along +/-Y (wrist_link's X turned FlangeParams.pulley_bolt_deg, with the pattern, where the arm mounts them).
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.params import M4_NUT, PULLEY_NUTS_MASS_G, pulley_90t_bolt_points

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = PULLEY_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(pulley_90t_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "pressed into the hex channels in wrist_link's flange from the pancake's pocket, before the pancake"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, each with a corner along +/-Y: the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.pi / 2.0, xy) for xy in pulley_90t_bolt_points())


@step(out="wrist_pulley_nuts.step")
def wrist_pulley_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_pulley_nuts()   # build: writes the sibling wrist_pulley_nuts.step
