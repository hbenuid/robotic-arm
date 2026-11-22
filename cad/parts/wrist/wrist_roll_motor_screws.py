"""wrist_roll_motor_screws - purchased (COTS) part: the wrist-roll pancake's 4 M3 socket head cap screws (ISO 4762).

They hold the pancake (nema17_pancake:wrist_roll) on wrist_link's plate - the pocket's front wall, slotted along Y for
the belt's tension (no idler): the heads in the counterbores round the slots in the plate's front face
(WristRollParams pad_cbore_*), bearing on their floors and flush with the face, the shanks through the slots into the
pancake's tapped holes, pad_screw_thread of thread in them. Length (under the head): WristRollParams.pad_screw_len (the
plate under the counterbores and that thread). Fit them once the pancake is forward in its pilot slot, from
the front face (the 20T clear inside them); slacken them to tension the belt.

In the arm: x1, in wrist_roll_drive (its row in assemblies/wrist_roll_drive.py, the stator - wrist_pitch_link). The board's
kit screws (mks_servo42d:wrist_roll, M3 x 30) reach the same holes from behind: their tips 0.6 into these screws' thread in
the model (docs/open_issues.md: the board's screws to buy on a pancake).

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/wrist/test_roll_screws.py locks its numbers. Frame: axis on Z, the
heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the slots' centres about the pancake's axis
(lib/wrist/layout.py pad_bolt_points() less the motor's axis - the module's row puts the origin on that axis, turned
over: +Z down R' into the motor).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import WRIST_ROLL_MOTOR_SCREWS_MASS_G
from lib.wrist import DEFAULT as WRIST
from lib.wrist.layout import pad_bolt_points, stack_positions

SCREW = WRIST.roll.pad_screw
LENGTH = WRIST.roll.pad_screw_len
_S = stack_positions(WRIST)
POINTS = tuple((x - _S["x_motor"], y - _S["y_motor"]) for x, y in pad_bolt_points(WRIST))

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_MOTOR_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the wrist-roll pancake onto wrist_link's plate: heads in the counterbores in its front face, "
                 "through the tension slots into the pancake's tapped holes")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="wrist_roll_motor_screws.step")
def wrist_roll_motor_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_motor_screws()   # build: writes the sibling wrist_roll_motor_screws.step
