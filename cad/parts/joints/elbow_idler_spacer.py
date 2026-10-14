"""elbow_idler_spacer - the printed spacer of the elbow belt's tensioner (parametric build123d, lib/upper_arm/link.py
build_idler_spacer): a stepped sleeve on the idler's axle between the idler and j1_link's slab.

j1_link prints on its slab's underside (lib/upper_arm/params.py ArmParams), so the riser the roll frame grows under its
idler (lib/forearm/params.py RollIdlerParams) is a part of its own here: its riser (ElbowIdlerParams.riser_dia) on the
slab's underside, its land (land_dia x land_h) on the idler's bearing inner ring only - the idler's flanges clear of
the riser -, its bore the axle's slot (slot_w); J1_IDLER_SPACER_H tall, so the idler hangs level with the belt
(lib/params.py). Frame: axis on Z, the land's face on z=0, the riser's on z=J1_IDLER_SPACER_H. Prints riser down.

In the arm: x1 (elbow_idler_spacer#1, a mount on j1_link#1 - lib/mounts.py IDLER_MOUNTS).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/upper_arm/test_elbow_idler.py.
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.params import J1_IDLER_SPACER_H
from lib.upper_arm import DEFAULT
from lib.upper_arm.link import build_idler_spacer

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/upper_arm/test_elbow_idler.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (on the idler's axle)


@step
def elbow_idler_spacer():
    """The spacer at its local origin (the land's face on the axis); the assembly owns placement."""
    part = build_idler_spacer(J1_IDLER_SPACER_H, DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    elbow_idler_spacer()   # build: writes the sibling elbow_idler_spacer.step
