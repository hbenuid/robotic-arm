"""gt2_pulley_20t - purchased (COTS) part; source of truth is vendor/gt2_pulley_20t.step.

20-tooth GT2 pulley (SolidWorks 'GT2_20T', Russian-locale config 'Конфигурация1'); 3 solid bodies in the vendor file.

SolidWorks product: 'GT2_20T_Конфигурация1'
Source export:      step/GT2_20T_Конфигурация1.STEP
Reference: mm units, 3 solid(s), volume 1878.6 mm^3,
           bbox size (14.45, 16, 16) mm, bbox min (0, -8, -8) mm.
In the arm: x1 (gt2_pulley_20t#1).

COTS convention (parts/_cots_template.py): gen_step() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Align, Cylinder, Location, import_step  # noqa: E402

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 8.0     # [ESTIMATE] aluminium 20T GT2 pulley, 6 mm bore, ~8-10 g
VENDOR_STEP = pathlib.Path(__file__).resolve().parent.parent / "vendor" / f"{NAME}.step"


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: OD 16 cylinder built along +Z then rotated onto +X, placed at the reference bounding box.
    return Cylinder(8.0, 14.45, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0.0, 0.0, 0.0)))


def gen_step():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = import_step(str(VENDOR_STEP)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
