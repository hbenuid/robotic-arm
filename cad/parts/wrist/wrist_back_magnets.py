"""wrist_back_magnets - purchased (COTS) part: the wrist frame's back panel's magnets, 4 pairs of N52 NdFeB disc
magnets (lib/wrist/params.py WristBackPanelParams magnet_*).

Each pair holds the back panel (wrist_back_panel) to the frame (wrist_link): one disc glued into a pocket in the frame's
step on the rear, one into a pocket in the panel's inner face, each pressed to its pocket's bottom - the two facing each
other across the step's floor, attracting (the frame's all the same pole out, the panel's the other).

In the arm: x1, in wrist_roll_drive (its row in assemblies/wrist_roll_drive.py, the STATOR - wrist_pitch_link), built at
their stations (lib/wrist/roll.py back_magnet, back_magnet_depths), so the row is at 0.

No catalog model (none adopted), no export and NO reference file: a pattern part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (plain discs) and tests/wrist/test_back_panel.py locks its numbers.
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.params import WRIST_BACK_MAGNETS_MASS_G
from lib.wrist import DEFAULT as WRIST
from lib.wrist.roll import back_magnet, back_magnet_depths

BACK = WRIST.roll.back
POINTS = BACK.magnet_points

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_BACK_MAGNETS_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{BACK.magnet_dia:g} x {BACK.magnet_t:g} mm N52 NdFeB disc magnet"
PURCHASE_QTY = 2 * len(POINTS)   # pieces per occurrence (a pair at each point)
PURCHASE_NOTE = ("the wrist frame's back panel: one of each pair glued into the frame's step, one into the panel - "
                 "mark the poles first, so each pair attracts")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The eight plain discs, at their stations in the drive's module frame: the geometry until a vendor model exists."""
    return pattern(back_magnet(x, y, depth, WRIST) for x, y in POINTS for depth in back_magnet_depths(WRIST))


@step(out="wrist_back_magnets.step")
def wrist_back_magnets():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_back_magnets()   # build: writes the sibling wrist_back_magnets.step
