"""roll_wall_screws - purchased (COTS) part: the forearm wall's 4 M3 socket head cap screws (ISO 4762).

They bolt the forearm's wall (j2_wall_bracket) onto the forearm roll pulley (forearm_roll_pulley): from counterbores in
the wall's wrist face, through the wall and the pulley's front face into the nuts in the pulley's core (roll_wall_nuts) -
each one clamps the wall onto the ring's face (lib/forearm/params.py RollEndParams screw / head_* / screw_len).

In the arm: x1, in forearm_roll_drive (its row in assemblies/forearm_roll_drive.py, the ROTOR - forearm_link): the
heads in the counterbores, flush with the wall's wrist face (lib/forearm/layout.py stack_positions z_end_head), the
shanks down the M3 clearance holes through the wall into the pulley, the tips flush with the nuts' far faces
(z_end_tip). Length (under the head): RollEndParams.screw_len. Bolt the wall on before the wrist-pitch motor goes on
(the motor stands in front of the heads).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_roll_wall_screws.py locks its numbers. Frame: axis on
Z, the heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the wall's bolt circle about the
roll axis (lib/forearm/layout.py flange_bolt_points_module(), y mirrored: the module's row turns the pattern over
about X, the heads up the roll axis toward the wrist).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import flange_bolt_points_module
from lib.params import ROLL_WALL_SCREWS_MASS_G

SCREW = FOREARM.roll_end.screw
LENGTH = FOREARM.roll_end.screw_len
POINTS = tuple((x, -y) for x, y in flange_bolt_points_module(FOREARM))   # turned over by the row: module (x, y) = (x, -y) here

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_WALL_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the forearm wall onto the roll pulley: heads flush in the wall's wrist-face counterbores, through the "
                 "wall and the pulley's front face into the nuts in its core; bolt the wall on before the wrist-pitch motor")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="roll_wall_screws.step")
def roll_wall_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_wall_screws()   # build: writes the sibling roll_wall_screws.step
