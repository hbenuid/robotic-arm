"""TEMPLATE - an import WRAPPER: parts/<name>.py that returns the SolidWorks reference
geometry (reference/<name>.step) until the part is converted to parametric build123d.

This is the day-one state of every custom part. The part's LOCAL frame is the SolidWorks
part-file frame (identity LOCAL_FROM_REF): reference/placements.json places it in the arm
assuming exactly that frame, so the whole arm assembles before anything is converted.

To CONVERT the part:
  1. rewrite gen_step() with BuildPart/... pulling shared dims from lib.params;
  2. set CONVERTED = True; if you choose a nicer local origin, set LOCAL_FROM_REF to the
     rigid transform reference-frame -> new-local-frame (the assemblies compose
     placement * LOCAL_FROM_REF.inverse(), so placements.json never changes);
  3. ./cadtool pytest tests/test_reference_match.py -k <name>   (volume + bbox vs reference)
  4. ./cadtool step parts/<name>.py                             (regenerate the committed STEP)
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
# Optional per-part tolerances for tests/test_reference_match.py:
# REF_VOL_TOL = 0.005   (relative)     REF_BBOX_TOL = 0.2   (mm)


def gen_step():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
