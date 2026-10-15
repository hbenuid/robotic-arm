"""wrist_roll_shaft - the back piece of the wrist roll drive's rotor, a collar behind bearing 1: its hub a Ø30.3 journal
through bearing 1 (the rear 6806) to the lip's middle, where the pulley's hub meets it, the Ø33 shoulder on bearing 1's
inner ring, the Ø36 flange in the frame's bay with the rotor clamp's 4 nuts (pockets open to its rear face and the bore)
and the hard-stop lug that meets the frame's post; the Ø10 cable bore.

Designed here, no reference: tests/wrist/test_roll_drive.py holds its numbers (lib/wrist/roll.py build_shaft(cfg),
every number lib/wrist/params.py WristRollParams, in the drive's MODULE frame at its stack station -
assemblies/wrist_roll_drive.py places it at 0). PETG, printed rear face down. In the arm: x1, inside wrist_roll_drive#1 -
wrist_roll_link.
"""
import pathlib

from cadgen import step

from lib.wrist import DEFAULT, WristConfig
from lib.wrist.roll import build_shaft

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: WristConfig = DEFAULT):
    return build_shaft(cfg)


@step
def wrist_roll_shaft():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_roll_shaft()   # build: writes the sibling wrist_roll_shaft.step
