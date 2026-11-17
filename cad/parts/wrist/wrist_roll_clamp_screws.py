"""wrist_roll_clamp_screws - purchased (COTS) part: the wrist roll rotor clamp's 4 M3 socket head cap screws (ISO 4762).

The forearm roll's clamp (parts/joints/roll_clamp_screws) one joint further out: from the counterbores in the wrist roll
pulley's face (wrist_roll_pulley) through its hub, both 6806 inner rings and the shaft (wrist_roll_shaft) into the nuts
in the shaft's flange (wrist_roll_clamp_nuts) - the pulley, both inner rings and the shaft one body. Tighten them before
gripper_j3_connector goes on (it covers their heads).

In the arm: x1, in wrist_roll_drive (its row in assemblies/wrist_roll_drive.py, the rotor - wrist_roll_link): the heads
on the counterbores' floor, clamp_head_clear under the pulley's face, the tips at the shaft's rear face
(WristRollParams.clamp_screw_len sets that face). Length (under the head): WristRollParams.clamp_screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md), no SolidWorks export and NO reference
file: a pattern part with no reference (lib/reference.py NO_REFERENCE) - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and tests/wrist/test_roll_screws.py locks its numbers. Frame: axis on Z, the
heads' bearing face on z=0 (the heads in -Z), the shanks in +Z; the four on the clamp's circle (lib/wrist/layout.py
clamp_points() mirrored in y: the module's row turns the pattern over about X, the heads up R').
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import WRIST_ROLL_CLAMP_SCREWS_MASS_G
from lib.wrist import DEFAULT as WRIST
from lib.wrist.layout import clamp_points

SCREW = WRIST.roll.clamp_screw
LENGTH = WRIST.roll.clamp_screw_len
POINTS = tuple((x, -y) for x, y in clamp_points(WRIST))   # turned over by the row: module (x, y) = (x, -y) here

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = WRIST_ROLL_CLAMP_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{SCREW.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(POINTS)   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = ("the wrist roll rotor's clamp: heads in the pulley face's counterbores, through both 6806s into the nuts "
                 "in the shaft's flange; tighten before the gripper's connector goes on")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(SCREW, LENGTH, xy) for xy in POINTS)


@step(out="wrist_roll_clamp_screws.step")
def wrist_roll_clamp_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    wrist_roll_clamp_screws()   # build: writes the sibling wrist_roll_clamp_screws.step
