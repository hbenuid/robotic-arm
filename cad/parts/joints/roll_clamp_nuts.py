"""roll_clamp_nuts - purchased (COTS) part: the forearm roll rotor clamp's 4 M3 hex nuts (ISO 4032), in the roll shaft.

The rotor clamp's screws (roll_clamp_screws) run into them from the pulley's front face.

In the arm: x1, in forearm_roll_drive (its row in assemblies/forearm_roll_drive.py, the ROTOR - forearm_link): each nut
in its pocket in the shaft's flange (forearm_roll_shaft; lib/forearm/layout.py clamp_nut_pocket) - a radial slot open
to the shaft's rear face and into the cable bore, a flat either side -, its bearing face up against the pocket's end
(stack_positions z_clamp_nut_seat), 2 pitches of the screw past its far face; a corner radial, as the slot holds it, so it
cannot turn. Pushed into the pockets from the shaft's rear face (in the frame's bay) before the screws go in.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/forearm/test_roll_clamp.py locks its numbers.
Frame: the module's (the row carries the far faces' station only): axes on Z on the clamp's circle about the roll axis
(lib/forearm/layout.py clamp_points()), the far faces on z=0, standing in +Z, a corner along each nut's radius.
"""
import math
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import clamp_points
from lib.params import ROLL_CLAMP_NUTS_MASS_G

NUT = FOREARM.drive.clamp_nut
POINTS = tuple(clamp_points(FOREARM))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_CLAMP_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "in the roll shaft's flange, for the rotor clamp: pushed into the pockets from the shaft's rear face (in the frame's bay)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, a corner along each one's radius: the geometry until a vendor model exists."""
    return pattern(hex_nut(NUT, math.atan2(y, x), (x, y)) for x, y in POINTS)


@step
def roll_clamp_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_clamp_nuts()   # build: writes the sibling roll_clamp_nuts.step
