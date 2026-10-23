"""roll_clamp_screws - purchased (COTS) part: the forearm roll rotor clamp's 4 M3 socket head cap screws (ISO 4762).

They clamp the roll's rotor as one: from the pulley's front face (forearm_roll_pulley) through its hub, both 6806 inner
rings and the shaft (forearm_roll_shaft) into the nuts in the shaft's flange (roll_clamp_nuts) - the pulley, both inner
rings and the shaft one body, the drive whole without the forearm (lib/forearm/params.py RollDriveParams clamp_*).

In the arm: x1, in forearm_roll_drive (its row in assemblies/forearm_roll_drive.py, the ROTOR - forearm_link): the heads
in the counterbores in the pulley's front face (clamp_head_clear round and over them, under the forearm wall's recess),
the shanks down the M3 clearance holes through both pieces, the tips at the shaft's rear face (which the length sets:
lib/forearm/layout.py stack_positions z_shaft_end). Length (under the head): RollDriveParams.clamp_screw_len. Tighten
them before the forearm wall goes on (docs/forearm_roll.md section 4).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/forearm/test_roll_clamp.py locks its numbers. Frame: axis on Z, the
heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the clamp's circle about the roll axis
(lib/forearm/layout.py clamp_points(), y mirrored: the module's row turns the pattern over about X, the heads up the
roll axis toward the wrist).
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import clamp_points
from lib.params import ROLL_CLAMP_SCREWS_MASS_G

SCREW = FOREARM.drive.clamp_screw
LENGTH = FOREARM.drive.clamp_screw_len
POINTS = tuple((x, -y) for x, y in clamp_points(FOREARM))   # turned over by the row: module (x, y) = (x, -y) here

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_CLAMP_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the roll rotor's clamp: heads in the pulley's front-face counterbores, through both 6806s into the nuts "
                 "in the shaft's flange; tighten before the forearm wall goes on")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step
def roll_clamp_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_clamp_screws()   # build: writes the sibling roll_clamp_screws.step
