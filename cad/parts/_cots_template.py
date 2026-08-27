"""TEMPLATE - a PURCHASED (COTS) part. Copy to parts/<name>.py and edit.

The source of truth is the vendor STEP / datasheet, not this Python:
  * declare COTS = True and MASS_G = <datasheet grams>;
  * gen_step() is HYBRID: it returns vendor/<name>.step when that file exists, else a
    parametric ENVELOPE from lib.params - dropping a real STEP into cad/vendor/ (e.g. via
    /cad:step-parts) upgrades the part to exact geometry with no code change;
  * keep mating-critical dims (bolt pattern, shaft, bore) in lib.params so the designed
    parts that mate to it import the SAME numbers.
Local frame: for parts seeded from the SolidWorks exports it is the SolidWorks frame
(reference/placements.json assumes it) - transform a differently oriented vendor file
inside gen_step() to keep that frame.

Generate the committed STEP:   ./cadtool step parts/<name>.py
"""
# --- path shim -------------------------------------------------------------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Align, Box, import_step  # noqa: E402  (import after shim)
# from lib.params import ...                     # noqa: E402  the real interface dims

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = 0.0   # [DATASHEET] grams
VENDOR_STEP = pathlib.Path(__file__).resolve().parent.parent / "vendor" / f"{NAME}.step"


def _envelope():
    """Parametric stand-in built from lib.params - replace the placeholder box."""
    return Box(10, 10, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))


def gen_step():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    part = import_step(str(VENDOR_STEP)) if VENDOR_STEP.exists() else _envelope()
    part.label = NAME
    return part


# --- preview: guarded so importing this part has NO side effects ----------------------
if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
