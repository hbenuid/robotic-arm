"""wrist_idler_spacer - the printed spacer of the wrist belt's tensioner (parametric build123d, lib/forearm/link.py
build_idler_spacer): a stepped sleeve on the idler's axle between the idler and j2_link's web.

j2_link prints on its web's underside (lib/forearm/params.py WallSplitParams), so no rib grows from it under the idler:
the riser is a part of its own, as on the elbow (parts/joints/elbow_idler_spacer) and base_yaw - its riser
(IdlerParams.riser_dia) on the web's underside, its land (land_dia x land_h) on the idler's bearing inner ring only -
the idler's flanges clear of the riser -, its bore the axle's slot (slot_w); lib/forearm/layout.py
wrist_idler_spacer_h() tall, so the idler hangs level with the belt. It slides along the web's slot with the axle.
Frame: axis on Z, the land's face on z=0, the riser's on z=wrist_idler_spacer_h(). Prints riser down.

In the arm: x1 (wrist_idler_spacer#1, a mount on j2_link#1 - lib/mounts.py WRIST_IDLER_MOUNTS).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/forearm/test_wrist_idler.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT
from lib.forearm.link import build_idler_spacer

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: a small separate print, an idler's spacer
REFERENCE = None              # no reference file: tests/forearm/test_wrist_idler.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (on the idler's axle)


@step(out="wrist_idler_spacer.step")
def wrist_idler_spacer():
    """The spacer at its local origin (the land's face on the axis); the assembly owns placement."""
    part = build_idler_spacer(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_idler_spacer()   # build: writes the sibling wrist_idler_spacer.step
