"""roll_home_carrier - the printed carrier of the forearm_roll home sensor (parametric build123d, lib/forearm/roll_home.py
build_home_carrier): it holds the KY-003 hall module (ky003_hall_sensor) behind the roll shaft, in the roll frame's bay
under the back panel, its chip facing the magnet pressed into the shaft's rear face (roll_home_magnet).

The board's plate under the KY-003's sensor end - the board on it chip up on 2 M2 (roll_home_board_screws,
roll_home_board_nuts) through its own holes, the heads under the plate -, a leg back out of the bay's bore, a strip down
the back panel's opening's floor to its bottom: there a tube plugs into the frame's wire hole (it locates the carrier;
the sensor's lead runs down through it into the motor's pocket) and a notch takes the panel's rim, so the closed panel
holds the carrier on the floor. No screw into the frame (the user's choice). Every number: lib/forearm/params.py
RollHomeParams. Frame: forearm_roll_drive's module frame (built at its station). Prints on its -X side: the tube along
the bed, the M2 holes teardrops.

In the arm: x1 (roll_home_carrier, a row of forearm_roll_drive#1 - the stator, elbow_link).

No SolidWorks export and NO reference file: designed here, its own test locks its numbers -
tests/forearm/test_roll_home.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT
from lib.forearm.roll_home import build_home_carrier

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/forearm/test_roll_home.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled at its station in the module frame


@step(out="roll_home_carrier.step")
def roll_home_carrier():
    """The carrier at its station; the drive owns placement."""
    part = build_home_carrier(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    roll_home_carrier()   # build: writes the sibling roll_home_carrier.step
