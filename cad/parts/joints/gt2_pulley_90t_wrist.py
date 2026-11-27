"""gt2_pulley_90t_wrist - the printed 90T GT2 pulley of the wrist belt (parametric build123d, lib/pulley/body.py
build_pulley(WRIST), in the 90T's part frame - origin on the axis at the lower flange's top, +Y up the axis toward the
outer face).

The elbow's 90T (parts/joints/gt2_pulley_90t) with its 4 head counterbores GT2_PULLEY_90T_WRIST_HEAD_SEAT deep, 2
shallower: the wrist's nuts sit in wrist_link's plate, the wrist roll pancake 1.25 under it, so the wrist's stack is 2
shorter than the elbow's on the nut side and the heads stand 2 higher - 1 proud of the outer face - for the wrist to
take the elbow's M4 x 35 uncut (wrist_pulley_screws), its tips where a cut M4 x 33's were. Every other number is the
90T's: lib/pulley/params.py (WRIST).

Keep the two apart at assembly: the elbow's 90T on the wrist puts the M4 x 35's tips into the pancake. This one's heads
stand proud of its face.

In the arm: x1 (gt2_pulley_90t_wrist#1, a mount that re-seats the SolidWorks gt2_pulley_90t#2, retired,
PULLEY_SEAT_SHIFT out along the axis: its journal in the lower wrist 6806, its ring under that bearing's inner ring;
turned about its axis with wrist_link's bolt pattern - lib/mounts.py).

No SolidWorks export of its own and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks
its numbers - tests/pulley/test_gt2_pulley_90t_wrist.py. PETG, printed face down like the 90T.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.pulley import WRIST
from lib.pulley.body import build_pulley

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: a pulley
FINE_LAYERS = True            # printed at lib/params.py LAYER_H_FINE: its GT2 teeth
REFERENCE = None              # no reference file: tests/pulley/test_gt2_pulley_90t_wrist.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the 90T's part frame


@step(out="gt2_pulley_90t_wrist.step")
def gt2_pulley_90t_wrist():
    """The pulley at its local origin (the lower flange's top on the axis); the assembly owns placement."""
    part = build_pulley(WRIST)
    part.label = NAME
    return part


if __name__ == "__main__":
    gt2_pulley_90t_wrist()   # build: writes the sibling gt2_pulley_90t_wrist.step
