"""gt2_pulley_20_60t - the printed 20-60T compound GT2 pulley, meant for the elbow drive's second stage (parametric
build123d, lib/pulley/body.py build_compound(cfg), in the SolidWorks part frame - origin on the axis at the 60T band's
bottom, +Y up the axis toward the 20T band).

A 60T band and a 20T band on one Ø8 bore, each band (the GT2 groove of lib/belts.py, lib/pulley/teeth.py) between two
flanges chamfered on the teeth's side, the 20T's lower flange standing on the 60T's upper flange; solid - no hub, no
web, no set screw. Every number: lib/pulley/params.py (CompoundPulleyParams).

SolidWorks product: 'GT2 Pulley - 20 - 60 teeth'
Source export:      'GT2 Pulley - 20 - 60 teeth.STEP' (1 974 574 bytes, sha256 55733855bdc5..., lib/reference.py MEASURED)
Export: mm units, 1 solid, volume 10693.26 mm^3,
        bbox size (40.089, 18.8, 40.089) mm, bbox min (-20.045, -1.2, -20.045) mm.

A measured conversion (parts/AGENTS.md Part states): the export was measured once and is not committed - no reference
file, no manifest entry; tests/pulley/test_gt2_pulley_20_60t.py locks the build to the export's numbers (its arc
centres, surfaces, volume, bbox). NOT PLACED yet (UNPLACED below, parts/AGENTS.md "Modelled, not placed yet").
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.pulley import COMPOUND
from lib.pulley.body import build_compound

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # a measured conversion: no reference file (lib/reference.py MEASURED)
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame
UNPLACED = ("the elbow drive's second stage is not designed yet - an 8 mm shaft in j1_link's x 128 seats, its belts and "
            "the elbow motor's new place; that design places it (docs/open_issues.md)")


@step
def gt2_pulley_20_60t():
    """The pulley at its local origin (the 60T band's bottom on the axis); the assembly owns placement."""
    part = build_compound(COMPOUND)
    part.label = NAME
    return part


if __name__ == "__main__":
    gt2_pulley_20_60t()   # build: writes the sibling gt2_pulley_20_60t.step
