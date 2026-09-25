"""mg996r_servo - purchased (COTS) part; source of truth is vendor/mg996r_servo.step.

TowerPro MG996R servo, third-party 2020 model (4 solid bodies in the vendor file).

SolidWorks product: 'Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model'
Source export:      step/Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model.STEP
Reference: mm units, 4 solid(s), volume 33161.3 mm^3,
           bbox size (55.8, 45.2, 20.5) mm, bbox min (-28.5, -0, -10.25) mm.
In the arm: x1 (mg996r_servo#1).

COTS convention (parts/_templates/cots.py): mg996r_servo() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import read_step, step

from lib.datum import IDENTITY, to_location
from lib.params import MG996R_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = MG996R_MASS_G   # [DATASHEET] 55 g
PURCHASE_SPEC = "TowerPro MG996R servo"
PURCHASE_QTY = 1    # pieces per occurrence
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = IDENTITY


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: the reference bounding box (tabs along X, height along Y, thickness along Z).
    return bd.Box(55.8, 45.2, 20.5, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN)).moved(bd.Location((-28.5, -0.0, -10.25)))


@step
def mg996r_servo():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = read_step(VENDOR_STEP).moved(to_location(VENDOR_TO_REF)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


if __name__ == "__main__":
    mg996r_servo()   # build: writes the sibling mg996r_servo.step
