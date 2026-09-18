"""TEMPLATE - an import WRAPPER: parts/<group>/<name>.py that returns the SolidWorks reference
geometry (reference/solidworks/<name>.step) until the part is converted to parametric build123d.

This is the day-one state of every custom part. The part's LOCAL frame is the SolidWorks
part-file frame (identity LOCAL_FROM_REF): reference/placements.json places it in the arm
assuming exactly that frame, so the whole arm assembles before anything is converted.

The file declares ONE cadgen model: `@step def <name>()` (rename `wrapper` to the part NAME when
you copy the template - NAME = file stem = model name); `./cadtool gen parts/<group>/<name>.py`
runs it and writes the sibling <name>.step. Tests call parts.build(name), never the model.

To CONVERT the part:
  1. rewrite the model body with BuildPart/... pulling shared dims from lib.params;
  2. set CONVERTED = True; if you choose a nicer local origin, set LOCAL_FROM_REF to the
     rigid transform reference-frame -> new-local-frame as DATA, ((x, y, z), (rx, ry, rz)) in mm /
     degrees (the assemblies compose placement * to_location(LOCAL_FROM_REF).inverse(), so
     placements.json never changes); import the kernel lazily - see _templates/designed.py;
  3. ./cadtool pytest tests/test_reference_match.py -k <name>   (volume + bbox vs reference)
  4. ./cadtool gen parts/<group>/<name>.py                              (regenerate the committed STEP)
"""
import pathlib

from cadgen import step
from lib import reference
from lib.datum import IDENTITY, to_location

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once the model is parametric build123d
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (identity = SolidWorks frame)
# Optional per-part tolerances for tests/test_reference_match.py:
# REF_VOL_TOL = 0.005   (relative)     REF_BBOX_TOL = 0.2   (mm)


@step
def wrapper():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(to_location(LOCAL_FROM_REF))
    shape.label = NAME
    return shape


if __name__ == "__main__":
    wrapper()   # build: writes the sibling wrapper.step (preview: ./cadtool show parts/_templates/wrapper.py)
