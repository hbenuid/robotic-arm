"""shoulder_home_tray - the printed tray of the shoulder_pitch home sensor (parametric build123d, lib/yaw_coupler/body.py
build_home_tray): dropped into j1_coupler's pocket over the hub, under the drive's turning shell, it holds the KY-003 hall
module (ky003_hall_sensor) under a magnet glued on a pillar of the shell (shoulder_home_magnet).

A plate filling the pocket's floor - the pocket's walls place it, so home is set by the print -, holed round the tips of
the 120T's 4 M4s, a finger slot at its back end to lift it out; on it a block whose top is the board's plate, facing
the drive's axis; the board on it on 2 M2 (shoulder_home_board_screws, shoulder_home_board_nuts) through its own holes,
the heads in recesses in the block; relief slots under the header's pins and the chip's leads. Held by its fit (the
coupler only turns about the vertical base_yaw axis: it never tips out) - a dab of glue against rattling. Every number:
lib/yaw_coupler/params.py HomeTrayParams. Frame (lib/yaw_coupler/layout.py home_tray_frame()): z = 0 on the pocket's
floor, +Z up the coupler's +Y, the coupler's (x, z) at (x, -y). Prints on its underside.

In the arm: x1 (shoulder_home_tray#1, a mount on j1_coupler#1 - lib/mounts.py SHOULDER_HOME_MOUNTS).

No SolidWorks export and NO reference file: designed here, its own test locks its numbers -
tests/base/test_shoulder_home.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.yaw_coupler import DEFAULT
from lib.yaw_coupler.body import build_home_tray

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/base/test_shoulder_home.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (home_tray_frame())


@step(out="shoulder_home_tray.step")
def shoulder_home_tray():
    """The tray at its local origin (its underside on z = 0); the assembly owns placement."""
    part = build_home_tray(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    shoulder_home_tray()   # build: writes the sibling shoulder_home_tray.step
