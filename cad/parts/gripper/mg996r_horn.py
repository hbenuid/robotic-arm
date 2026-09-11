"""mg996r_horn - purchased (COTS) part; source of truth is vendor/mg996r_horn.step.

MG996R servo horn (ships with the servo).

SolidWorks product: 'Servo MG996R Horn_Servo MG996R Horn'
Source export:      step/Servo MG996R Horn_Servo MG996R Horn.STEP
Reference: mm units, 1 solid(s), volume 531.8 mm^3,
           bbox size (32, 2.5, 12) mm, bbox min (-16, 0, -6) mm.
In the arm: x1 (mg996r_horn#1).

COTS convention (parts/_templates/cots.py): mg996r_horn() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
import pathlib

from build123d import Align, Box, Location
from cadgen import read_step, step

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 3.0     # [ESTIMATE] nylon horn
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = Location()


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: the reference bounding box.
    return Box(32.0, 2.5, 12.0, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((-16.0, 0.0, -6.0)))


@step
def mg996r_horn():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = read_step(VENDOR_STEP).moved(VENDOR_TO_REF) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


if __name__ == "__main__":
    mg996r_horn()   # build: writes the sibling mg996r_horn.step (preview: ./cadtool show parts/gripper/mg996r_horn.py)
