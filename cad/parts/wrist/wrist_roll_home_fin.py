"""wrist_roll_home_fin - the printed fin of the wrist_roll home sensor (parametric build123d, lib/wrist/home.py
build_home_fin): it holds the KY-003 hall module (ky003_hall_sensor) upright beside the wrist roll pulley's face, on the
far face's side of wrist_link, its chip facing the magnet glued on the face's rim (wrist_roll_home_magnet).

The upright plate the board stands on - on 2 M2 from its component side (wrist_roll_home_board_screws) through its own
holes into nuts in hex pockets in the fin's back (wrist_roll_home_board_nuts) -, its back flush with the far face; under
it the shoe over the front facet, which sits on the facet, the front face and the bearing boss's top and nests against
the boss's round side (that places it), 0.2 off each for the glue. Glued on: nothing on the arm changes (the user's
choice). Every number: lib/wrist/params.py WristRollHomeParams. Frame: wrist_roll_drive's module frame (built at its
station). Prints on its back (the far face's side down): the M2 holes and the nut pockets upright, nothing overhanging.

In the arm: x1 (wrist_roll_home_fin, a row of wrist_roll_drive#1 - the stator, wrist_pitch_link).

No SolidWorks export and NO reference file: designed here, its own test locks its numbers -
tests/wrist/test_roll_home.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.wrist import DEFAULT
from lib.wrist.home import build_home_fin

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/wrist/test_roll_home.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled at its station in the module frame


@step(out="wrist_roll_home_fin.step")
def wrist_roll_home_fin():
    """The fin at its station; the drive owns placement."""
    part = build_home_fin(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_roll_home_fin()   # build: writes the sibling wrist_roll_home_fin.step
