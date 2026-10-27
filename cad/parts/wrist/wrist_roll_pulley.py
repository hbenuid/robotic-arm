"""wrist_roll_pulley - the wrist roll drive's OUTPUT, the front piece of its rotor: its hub a Ø30.3 journal through
bearing 2 (the front 6806) to the lip's middle, where the shaft's hub meets it, the Ø33 shoulder on bearing 2's inner
ring, the 40T GT2 band (two flanges) the pancake's 20T drives 2 : 1 on its Ø23 core, and the Ø42 face gripper_j3_connector
bolts to - 4x M3 into nuts in pockets from the face's rim; the Ø10 cable bore (the gripper servo's leads); the rotor
clamp's 4x M3 on Ø17 from counterbores in the face through both hubs into the shaft's nuts.

Designed here, no reference: tests/wrist/test_roll_drive.py holds its numbers (lib/wrist/roll.py build_pulley(cfg),
every number lib/wrist/params.py WristRollParams, in the drive's MODULE frame at its stack station -
assemblies/wrist_roll_drive.py places it at 0). PETG, printed face down. In the arm: x1, inside wrist_roll_drive#1 -
wrist_roll_link.
"""
import pathlib

from cadgen import step

from lib.wrist import DEFAULT, WristConfig
from lib.wrist.roll import build_pulley

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: WristConfig = DEFAULT):
    return build_pulley(cfg)


@step(out="wrist_roll_pulley.step")
def wrist_roll_pulley():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_roll_pulley()   # build: writes the sibling wrist_roll_pulley.step
