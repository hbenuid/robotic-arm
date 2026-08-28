"""j2_cap_2 - import wrapper around the SolidWorks reference (reference/j2_cap_2.step).

SolidWorks product: 'cap of joint 2 piece 2 8526'
Source export:      step/cap of joint 2 piece 2 8526.STEP
Reference: mm units, 1 solid(s), volume 75151.4 mm^3,
           bbox size (223.377, 90, 13.5) mm, bbox min (-753.703, 885.488, -13.5) mm.
In the arm: x1 (j2_cap_2#1).

Geometry sits far from the part origin (assembly coordinates baked into the part file); placements.json compensates - pick a sane origin via LOCAL_FROM_REF when converting.

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
