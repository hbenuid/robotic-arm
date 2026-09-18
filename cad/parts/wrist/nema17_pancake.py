"""nema17_pancake - purchased (COTS) part; source of truth is vendor/nema17_pancake.step.

NEMA 17 pancake stepper. The SolidWorks model is a 7-part sub-assembly of the motor's
internals (stator, rotor, plates, connector, 2x 625ZZ, 4x M3); tools/reference/extract_placements.py
flattens it into vendor/nema17_pancake.step (11 solids) in the sub-assembly's own frame.

SolidWorks product: 'nema17_pancake'
Source export:      (extracted from the full-assembly STEP)
Reference: mm units, 11 solid(s), volume 33013.8 mm^3,
           bbox size (41.5, 47, 43) mm, bbox min (-20.75, -13.591, 12.659) mm.
In the arm: x1 (nema17_pancake#1).

COTS convention (parts/_templates/cots.py): nema17_pancake() returns the vendor STEP when present,
else the parametric envelope below - both in the SolidWorks frame placements.json assumes.
"""
import pathlib

from cadgen import build123d as bd
from cadgen import read_step, step
from lib.datum import IDENTITY, to_location
from lib.params import PANCAKE_BODY_D, PANCAKE_BODY_H, PANCAKE_BODY_W, PANCAKE_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = PANCAKE_MASS_G   # [ESTIMATE] see lib/params.py
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
# Rigid transform vendor-file frame -> the SolidWorks frame placements.json assumes (identity while
# vendor/<name>.step is the SolidWorks re-export; set it after swapping in a step.parts model).
VENDOR_TO_REF = IDENTITY


def _envelope():
    """Parametric stand-in used only when the vendor STEP is missing."""
    # Envelope: body box from lib.params at the reference bounding box.
    return bd.Box(PANCAKE_BODY_W, PANCAKE_BODY_D, PANCAKE_BODY_H, align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN)).moved(bd.Location((-20.75, -13.591, 12.659)))


@step
def nema17_pancake():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = read_step(VENDOR_STEP).moved(to_location(VENDOR_TO_REF)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


if __name__ == "__main__":
    nema17_pancake()   # build: writes the sibling nema17_pancake.step (preview: ./cadtool show parts/wrist/nema17_pancake.py)
