"""gt2_pulley_90t - import wrapper around the SolidWorks reference (reference/gt2_pulley_90t.step).

SolidWorks product: 'GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric'
Source export:      step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP
Reference: mm units, 1 solid(s), volume 24810.7 mm^3,
           bbox size (59.188, 21.4, 59.188) mm, bbox min (-29.594, -13.2, -29.594) mm.
In the arm: x2 (gt2_pulley_90t#1, gt2_pulley_90t#2).

Printed 90-tooth GT2 pulley (config 'GT2 Pulley - Parametric'); 842 faces, ~1 s to import. Used at J2 and J3.

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
