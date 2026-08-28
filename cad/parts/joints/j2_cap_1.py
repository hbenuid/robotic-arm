"""j2_cap_1 - import wrapper around the SolidWorks reference (reference/j2_cap_1.step).

SolidWorks product: 'cap 1 joint 2 8726'
Source export:      step/cap 1 joint 2 8726.STEP
Reference: mm units, 1 solid(s), volume 97819.5 mm^3,
           bbox size (245.461, 90, 14.5) mm, bbox min (-200.461, -45, -14.5) mm.
In the arm: x1 (j2_cap_1#1).

Geometry is offset from the part origin; placements.json compensates.

Not yet parametric: gen_step() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

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
