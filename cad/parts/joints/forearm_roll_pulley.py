"""forearm_roll_pulley - the forearm roll drive's OUTPUT, the front piece of its rotor: the integral 90T GT2 ring (two
flanges) on a Ø44 core, its rear flange turning sunk in the frame's cup, its hub a Ø30.3 journal down through bearing 2
(the front 6806) to the lip's middle, where the shaft's hub meets it; the Ø39.7 spigot on its front face in the forearm
wall's recess - the wall bolts onto the ring's face, 4x M3 on Ø31 into nuts in pockets in the core (pushed in from the
Ø18 cable bore); the rotor clamp's 4x M3 on Ø24.5 from counterbores in its front face through both hubs into the
shaft's nuts.

Designed here, no reference: tests/forearm/test_roll_drive.py holds its numbers (lib/forearm/roll.py
build_pulley(cfg), every number lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack
station - assemblies/forearm_roll_drive.py places it at 0). PETG, printed spigot down. In the arm: x1, inside
forearm_roll_drive#1 - forearm_link.
"""
import pathlib

from cadgen import step

from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_pulley

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: ForearmConfig = DEFAULT):
    return build_pulley(cfg)


@step
def forearm_roll_pulley():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_pulley()   # build: writes the sibling forearm_roll_pulley.step
