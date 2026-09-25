"""TEMPLATE - a DESIGNED (parametric) part. Copy to parts/<group>/<name>.py and edit.

cadgen convention: ONE module-level `@step def <name>()` (rename `designed` to the part NAME
when you copy the template - NAME = file stem = model name) that RETURNS the final Part/Compound
at the part's LOCAL origin - the assembly owns placement. Every part MUST:
  * return a valid, labelled solid/compound from the model (label == NAME);
  * pull shared dimensions from lib.params (never hard-code a shared value);
  * have no import side effects - the `__main__` call is the BUILD (writes the sibling STEP);
    tests and tools use parts.build(name) (the body in-process, nothing written);
  * import `lib` / `parts` plainly: cadtool, pytest and .env put cad/ on the import path;
  * keep the CAD kernel LAZY: `from cadgen import build123d as bd` and `bd.<name>` inside function
    bodies only - no kernel object in a module-level constant or an argument default (cadgen gates an
    unchanged model in ~0.1 s only while build123d / OCP are unloaded; tests/test_lazy_kernel.py).
    A frame the module declares is data: lib.datum.IDENTITY or ((x, y, z), (rx, ry, rz)).
If the part replaces a SolidWorks reference, keep REFERENCE / LOCAL_FROM_REF (see
_templates/wrapper.py) so tests/test_reference_match.py gates the conversion.

Generate the STEP:   ./cadtool gen parts/<group>/<name>.py
"""
import pathlib

from cadgen import build123d as bd
from cadgen import step

from lib.datum import IDENTITY
from lib.params import NUDGE  # noqa: F401

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # e.g. NAME when reference/<NAME>.step exists
CONVERTED = True
LOCAL_FROM_REF = IDENTITY   # reference frame -> this part's local frame (only if REFERENCE is set)

# [DESIGN] example dims - move anything shared into lib/params.py
width = 20.0
depth = 20.0
thickness = 3.0


@step
def designed():
    """Return the final Part at LOCAL origin (footprint centred, thickness along +Z)."""
    with bd.BuildPart() as bp:
        bd.Box(width, depth, thickness, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    part = bp.part
    part.label = NAME
    return part


if __name__ == "__main__":
    designed()   # build: writes the sibling designed.step
