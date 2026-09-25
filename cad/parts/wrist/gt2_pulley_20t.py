"""gt2_pulley_20t - purchased (COTS) part; source of truth is vendor/gt2_pulley_20t.step.

20-tooth GT2 pulley (SolidWorks 'GT2_20T', Russian-locale config 'Конфигурация1'); 3 solid bodies in the vendor file.

SolidWorks product: 'GT2_20T_Конфигурация1'
Source export:      step/GT2_20T_Конфигурация1.STEP
Reference: mm units, 3 solid(s), volume 1878.6 mm^3,
           bbox size (14.45, 16, 16) mm, bbox min (0, -8, -8) mm.
In the arm: x2 (gt2_pulley_20t#1 on the wrist-roll pancake; one more inside forearm_roll_drive#1 on the roll motor).

COTS convention (parts/_templates/cots.py): gt2_pulley_20t() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import read_step, step
from lib.datum import IDENTITY, to_location
from lib.params import GT2_PULLEY_20T_TEETH

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 8.0     # [ESTIMATE] aluminium 20T GT2 pulley, 6 mm bore, ~8-10 g
PURCHASE_SPEC = f"GT2 {GT2_PULLEY_20T_TEETH}T timing pulley, 5 mm bore, 6 mm belt"
PURCHASE_QTY = 1    # pieces per occurrence
PURCHASE_NOTE = "bore and belt width read off the vendor model - confirm against the motor shaft"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = IDENTITY


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: OD 16 cylinder built along +Z then rotated onto +X, placed at the reference bounding box.
    return bd.Cylinder(8.0, 14.45, rotation=(0, 90, 0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0.0, 0.0, 0.0)))


@step
def gt2_pulley_20t():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = read_step(VENDOR_STEP).moved(to_location(VENDOR_TO_REF)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


if __name__ == "__main__":
    gt2_pulley_20t()   # build: writes the sibling gt2_pulley_20t.step
