"""TEMPLATE - a DESIGNED (parametric) part. Copy to parts/<group>/<name>.py and edit.

Plugin-native (cad@text-to-cad) convention: a module-level gen_step() that RETURNS the
final Part/Compound at the part's LOCAL origin - the assembly owns placement. Every part
MUST:
  * define gen_step() returning a valid, labelled solid/compound (the plugin's scripts/step
    imports this file and calls it);
  * pull shared dimensions from lib.params (never hard-code a shared value);
  * keep show()/export ONLY under `if __name__ == "__main__":` - importing must have
    no side effects;
  * keep the 2-line path shim so `from lib ...` resolves under Ctrl+F5, `python -m`,
    AND the plugin CLI.
If the part replaces a SolidWorks reference, keep REFERENCE / LOCAL_FROM_REF (see
_templates/wrapper.py) so tests/test_reference_match.py gates the conversion.

Generate the committed STEP:   ./cadtool step parts/<group>/<name>.py
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from build123d import Align, Box, BuildPart, Location  # noqa: E402  (import after shim)
from lib.params import NUDGE  # noqa: E402,F401

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # e.g. NAME when reference/<NAME>.step exists
CONVERTED = True
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (only if REFERENCE is set)

# [DESIGN] example dims - move anything shared into lib/params.py
width = 20.0
depth = 20.0
thickness = 3.0


def gen_step():
    """Return the final Part at LOCAL origin (footprint centred, thickness along +Z)."""
    with BuildPart() as bp:
        Box(width, depth, thickness, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part = bp.part
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
