"""j1_cap - import wrapper around the SolidWorks reference (reference/j1_cap.step).

SolidWorks product: 'first joint cap 8726'
Source export:      step/first joint cap 8726.STEP
Reference: inch units, 1 solid(s), volume 150356.6 mm^3,
           bbox size (300, 90, 27.228) mm, bbox min (-838.697, 855.589, -27.228) mm.
In the arm: x1 (j1_cap#1).

Inch-unit SolidWorks export (OCCT converts to mm). Geometry sits far from the part origin; placements.json compensates - pick a sane origin via LOCAL_FROM_REF when converting.

Not yet parametric: gen_step() returns the reference geometry in the SolidWorks part-file
frame. See parts/_wrapper_template.py for how to convert it to build123d.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Location  # noqa: E402  (import after shim)
from lib import reference       # noqa: E402

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gen_step() is parametric build123d
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (identity = SolidWorks frame)


def gen_step():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
