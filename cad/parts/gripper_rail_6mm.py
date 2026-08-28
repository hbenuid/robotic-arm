"""gripper_rail_6mm - purchased (COTS) part; source of truth is vendor/gripper_rail_6mm.step.

Round linear rail, 6 mm x 125 mm; used x2 in the gripper.

SolidWorks product: 'Gripper rail 6mm_Gripper rail 6mm'
Source export:      step/Gripper rail 6mm_Gripper rail 6mm.STEP
Reference: mm units, 1 solid(s), volume 3534.3 mm^3,
           bbox size (6, 125, 6) mm, bbox min (-3, 0, -3) mm.
In the arm: x2 (gripper_rail_6mm#1, gripper_rail_6mm#2).

COTS convention (parts/_cots_template.py): gen_step() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Align, Cylinder, Location, import_step  # noqa: E402
from lib.params import RAIL_DIA, RAIL_LEN  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 27.7    # [ESTIMATE] steel, from the reference volume (3534 mm^3 x 7.85e-3 g/mm^3)
VENDOR_STEP = pathlib.Path(__file__).resolve().parent.parent / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = Location()


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: round rail along Y - placed at the reference bounding box.
    return Cylinder(RAIL_DIA / 2, RAIL_LEN, rotation=(-90, 0, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0.0, 0.0, 0.0)))


def gen_step():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = import_step(str(VENDOR_STEP)).moved(VENDOR_TO_REF) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
