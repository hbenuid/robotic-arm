"""roll_back_magnets - purchased (COTS) part: the forearm roll frame's back panel's magnets, 4 pairs of N52 NdFeB disc
magnets (lib/forearm/params.py BackPanelParams magnet_*).

Each pair holds the back panel (forearm_roll_back_panel) to the frame (forearm_roll_block): one disc glued into a pocket
in the frame's step on the rear face, one into a pocket in the panel's inner face, each pressed to its pocket's bottom -
the two facing each other across the step's floor, attracting (the frame's all the same pole out, the panel's the other).

In the arm: x1, in forearm_roll_drive (its row in assemblies/forearm_roll_drive.py, the STATOR - elbow_link), built at
their stations (lib/forearm/roll.py back_magnet, back_magnet_depths), so the row is at 0.

No catalog model (none adopted), no export and NO reference file: a pattern part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (plain discs) and tests/forearm/test_roll_back_panel.py locks its numbers.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.forearm import DEFAULT as FOREARM
from lib.forearm.roll import back_magnet, back_magnet_depths
from lib.params import ROLL_BACK_MAGNETS_MASS_G

BACK = FOREARM.drive.back
POINTS = BACK.magnet_points

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_BACK_MAGNETS_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{BACK.magnet_dia:g} x {BACK.magnet_t:g} mm N52 NdFeB disc magnet"
PURCHASE_QTY = 2 * len(POINTS)   # pieces per occurrence (a pair at each point)
PURCHASE_NOTE = ("the roll frame's back panel: one of each pair glued into the frame's step, one into the panel - "
                 "mark the poles first, so each pair attracts")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The eight plain discs, at their stations in the drive's module frame: the geometry until a vendor model exists."""
    return pattern(back_magnet(x, s, depth, FOREARM) for x, s in POINTS for depth in back_magnet_depths(FOREARM))


@step(out="roll_back_magnets.step")
def roll_back_magnets():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    roll_back_magnets()   # build: writes the sibling roll_back_magnets.step
