"""roll_wall_nuts - purchased (COTS) part: the forearm wall's 4 M3 hex nuts (ISO 4032), in the roll pulley's core.

The forearm wall's screws (roll_wall_screws) run into them from the wall's wrist face.

In the arm: x1, in forearm_roll_drive (its row in assemblies/forearm_roll_drive.py, the ROTOR - forearm_link): each nut
in its pocket in the pulley's core (forearm_roll_pulley; lib/forearm/layout.py end_nut_pocket) - a radial slot open
into the cable bore, a flat either side -, its bearing face toward the wall on the pocket's end (stack_positions
z_end_nut_seat), its far face flush with the screw's tip (z_end_nut); a corner radial, as the slot holds it, so it
cannot turn. Pushed into the pockets from the pulley's cable bore before the wall goes on.

No catalog model (none adopted), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
hex_nut(): no thread, the bore at the nominal diameter) and tests/forearm/test_roll_wall_screws.py locks its numbers.
Frame: the module's (the row carries the far faces' station only): axes on Z on the wall's bolt circle about the roll
axis (lib/forearm/layout.py flange_bolt_points_module()), the far faces on z=0, standing in +Z, a corner along each
nut's radius.
"""
import math
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import hex_nut
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import flange_bolt_points_module
from lib.params import ROLL_WALL_NUTS_MASS_G

NUT = FOREARM.drive.end_nut
POINTS = tuple(flange_bolt_points_module(FOREARM))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_WALL_NUTS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{NUT.d:g} hex nut (ISO 4032)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "in the roll pulley's core, for the forearm wall's screws: pushed into the pockets from the pulley's cable bore before the wall goes on"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain nuts, a corner along each one's radius: the geometry until a vendor model exists."""
    return pattern(hex_nut(NUT, math.atan2(y, x), (x, y)) for x, y in POINTS)


@step(out="roll_wall_nuts.step")
def roll_wall_nuts():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_wall_nuts()   # build: writes the sibling roll_wall_nuts.step
