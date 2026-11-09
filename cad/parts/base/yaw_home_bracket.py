"""yaw_home_bracket - the printed bracket of the base_yaw home sensor (parametric build123d, lib/base/body.py
build_yaw_home_bracket): a plain plate under the base's motor plate that holds the KY-003 hall module
(ky003_hall_sensor) under the 120T, its chip under the home magnet's path (yaw_home_magnet).

Its root lies on the plate's underside on the base's +Z side, on 2 M3 (yaw_home_screws) up through the plate into nuts
in hex pockets in its top (yaw_home_nuts); it runs in under the 120T past the plate's opening to the board, which lies
on its top on 2 M2 (yaw_home_board_screws, yaw_home_board_nuts) through its own holes; relief slots under the header's
pins (through it) and the chip's leads. Every number: lib/base/params.py YawHomeParams. Frame (lib/base/layout.py
home_bracket_frame()): z = 0 on its underside, +Z up the base_yaw axis, the base's (x, z) at (x, -y). Prints on its
underside.

In the arm: x1 (yaw_home_bracket#1, a mount on base#1 - lib/mounts.py YAW_HOME_MOUNTS).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/base/test_yaw_home.py.
"""
import pathlib

from cad import step

from lib.base import DEFAULT
from lib.base.body import build_yaw_home_bracket
from lib.datum import IDENTITY

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/base/test_yaw_home.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (home_bracket_frame())


@step(out="yaw_home_bracket.step")
def yaw_home_bracket():
    """The bracket at its local origin (its underside on z = 0); the assembly owns placement."""
    part = build_yaw_home_bracket(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    yaw_home_bracket()   # build: writes the sibling yaw_home_bracket.step
