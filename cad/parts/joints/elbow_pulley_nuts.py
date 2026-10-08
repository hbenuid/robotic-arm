"""elbow_pulley_nuts - purchased (COTS) part: the elbow 90T's 4 M4 hex nuts (ISO 4032), captive in the elbow block.

The elbow 90T's screws run into them, in the block's hex channels (forearm_roll_block).

In the arm (lib/mounts.py, hosted on elbow_pulley_screws): each nut on its channel's seat (lib/forearm/params.py
RollDriveParams.nut_seat_x), a flat toward the elbow axis as the channel holds it (lib/forearm/roll.py); the channels
are nut_af across flats, a press for the nut (docs/open_issues.md). Each drops in from the block's motor pocket before
the motor goes in (docs/forearm_roll.md §4).

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like the drive's cycloidal_housing_nuts - its envelope IS the geometry
(lib/fasteners.py hex_nut(): no thread, the bore at the nominal diameter) and its reference is that envelope
(reference/native/elbow_pulley_nuts.step, tools/reference/import_native.py). Frame: axis on Z, the nuts' bearing faces
on z=0, standing in +Z; the four on the 90T's hole circle (lib/belts.py pulley_90t_bolt_points()), each a flat toward
the pattern's centre.
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
PURCHASE_NOTE = "captive in the roll frame's hex channels (dropped in from the motor's cradle before the roll motor)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, keyed radially (a corner 30 deg off each nut's radius): the geometry until a vendor model exists."""
    return pattern(hex_nut(M4_NUT, math.atan2(y, x) + math.pi / 6.0, (x, y)) for x, y in pulley_90t_bolt_points())


@step
def elbow_pulley_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    elbow_pulley_nuts()   # build: writes the sibling elbow_pulley_nuts.step
