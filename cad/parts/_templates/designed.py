"""TEMPLATE - a DESIGNED (parametric) part. Copy to parts/<group>/<name>.py and edit.

cadgen 0.5 convention: ONE module-level `@step def <name>()` (rename `designed` to the part NAME
when you copy the template - NAME = file stem = model name) that RETURNS the final Part/Compound
at the part's LOCAL origin - the assembly owns placement. Every part MUST:
  * return a valid, labelled solid/compound from the model (label == NAME);
  * pull shared dimensions from lib.params (never hard-code a shared value);
  * have no import side effects - the `__main__` call is the BUILD (writes the sibling STEP);
    previews are `./cadtool show parts/<group>/<name>.py`, tests use parts.build(name);
  * import `lib` / `parts` plainly: cadtool, pytest and .env put cad/ on the import path.
If the part replaces a SolidWorks reference, keep REFERENCE / LOCAL_FROM_REF (see
_templates/wrapper.py) so tests/test_reference_match.py gates the conversion.

Generate the committed STEP:   ./cadtool gen parts/<group>/<name>.py
"""
import pathlib

from build123d import Align, Box, BuildPart, Location
from cadgen import step
from lib.params import NUDGE  # noqa: F401

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # e.g. NAME when reference/<NAME>.step exists
CONVERTED = True
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (only if REFERENCE is set)

# [DESIGN] example dims - move anything shared into lib/params.py
width = 20.0
depth = 20.0
thickness = 3.0


@step
def designed():
    """Return the final Part at LOCAL origin (footprint centred, thickness along +Z)."""
    with BuildPart() as bp:
        Box(width, depth, thickness, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = bp.part
    part.label = NAME
    return part


if __name__ == "__main__":
    designed()   # build: writes the sibling designed.step (preview: ./cadtool show parts/_templates/designed.py)
