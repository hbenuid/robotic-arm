"""gripper_rail_6mm - purchased (COTS) part; source of truth is vendor/gripper_rail_6mm.step.

Round linear rail, 6 mm x 125 mm; used x2 in the gripper.

SolidWorks product: 'Gripper rail 6mm_Gripper rail 6mm'
Source export:      step/Gripper rail 6mm_Gripper rail 6mm.STEP
Reference: mm units, 1 solid(s), volume 3534.3 mm^3,
           bbox size (6, 125, 6) mm, bbox min (-3, 0, -3) mm.
In the arm: x2 (gripper_rail_6mm#1, gripper_rail_6mm#2).

COTS convention (parts/_templates/cots.py): gripper_rail_6mm() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import read_step, step

from lib.datum import IDENTITY, to_location
from lib.params import RAIL_DIA, RAIL_LEN

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 27.7    # [ESTIMATE] steel, from the reference volume (3534 mm^3 x 7.85e-3 g/mm^3)
PURCHASE_SPEC = f"{RAIL_DIA:g} mm round steel linear shaft, {RAIL_LEN:g} mm long"
PURCHASE_QTY = 1    # pieces per occurrence (the gripper places it twice)
PURCHASE_NOTE = "confirm material / tolerance (h6 linear shaft vs plain rod); cut to length"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = IDENTITY


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: round rail along Y - placed at the reference bounding box.
    return bd.Cylinder(RAIL_DIA / 2, RAIL_LEN, rotation=(-90, 0, 0), align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)).moved(bd.Location((0.0, 0.0, 0.0)))


@step
def gripper_rail_6mm():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = read_step(VENDOR_STEP).moved(to_location(VENDOR_TO_REF)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_rail_6mm()   # build: writes the sibling gripper_rail_6mm.step
