"""mg996r_servo - purchased (COTS) part; source of truth is vendor/mg996r_servo.step.

TowerPro MG996R servo, third-party 2020 model (4 solid bodies in the vendor file).

SolidWorks product: 'Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model'
Source export:      step/Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model.STEP
Reference: mm units, 4 solid(s), volume 33161.3 mm^3,
           bbox size (55.8, 45.2, 20.5) mm, bbox min (-28.5, -0, -10.25) mm.
In the arm: x1 (mg996r_servo#1).

COTS convention (parts/_cots_template.py): gen_step() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Align, Box, Location, import_step  # noqa: E402
from lib.params import MG996R_MASS_G  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = MG996R_MASS_G   # [DATASHEET] 55 g
VENDOR_STEP = pathlib.Path(__file__).resolve().parent.parent / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = Location()


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: the reference bounding box (tabs along X, height along Y, thickness along Z).
    return Box(55.8, 45.2, 20.5, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Location((-28.5, -0.0, -10.25)))


def gen_step():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = import_step(str(VENDOR_STEP)).moved(VENDOR_TO_REF) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
