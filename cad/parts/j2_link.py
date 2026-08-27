"""j2_link - import wrapper around the SolidWorks reference (reference/j2_link.step).

SolidWorks product: 'Joint 2 change 8126'
Source export:      step/Joint 2 change 8126.STEP
Reference: mm units, 1 solid(s), volume 285470.5 mm^3,
           bbox size (300, 90, 33.5) mm, bbox min (-255, -45, 0) mm.
In the arm: x1 (j2_link#1).

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
