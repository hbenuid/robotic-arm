"""roll_idler_spacer - the printed spacer of the roll belt's tensioner (parametric build123d, lib/forearm/roll.py
build_idler_spacer): a stepped sleeve on the idler's axle between the idler and the roll frame's front face.

forearm_roll_block prints front face down (lib/forearm/params.py RollDriveParams), so no riser grows from the face
under the idler: the riser is a part of its own, as on the elbow (parts/joints/elbow_idler_spacer), base_yaw and the
wrist - its riser (RollIdlerParams.riser_dia) on the frame's front face, its land (land_dia x land_h) under the idler's
bearing inner ring only - the idler's flanges clear of the riser -, its bore the axle's hole (slot_w);
lib/forearm/layout.py roll_idler_spacer_h() tall, so the idler stands level with the belt. It slides along the plate's
slot with the axle. Frame: axis on Z, the land's face on z=0, the riser's on z=roll_idler_spacer_h(). Prints riser down.

In the arm: x1, inside forearm_roll_drive#1 (assemblies/forearm_roll_drive.py, on the frame: the stator).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/forearm/test_roll_idler.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT
from lib.forearm.roll import build_idler_spacer

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: a small separate print, an idler's spacer
REFERENCE = None              # no reference file: tests/forearm/test_roll_idler.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (on the idler's axle)


@step(out="roll_idler_spacer.step")
def roll_idler_spacer():
    """The spacer at its local origin (the land's face on the axis); the assembly owns placement."""
    part = build_idler_spacer(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    roll_idler_spacer()   # build: writes the sibling roll_idler_spacer.step
