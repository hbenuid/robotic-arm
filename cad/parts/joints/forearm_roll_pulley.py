"""forearm_roll_pulley - the forearm roll drive's OUTPUT, the front piece of its rotor: the integral 90T GT2 ring (two
flanges) on a Ø44 core - on down past the rear flange to run_gap over the frame's front face, where bearing 2's front face
is flush -, its hub a Ø30.2 journal down through bearing 2
(the front 6806), then a Ø29.9 neck (bearing 2 slides over it) on through the tower's spacer to bearing 1's front face,
where the shaft's journal meets it; its front face the ring's, flat (no spigot), on the forearm wall's flat elbow
face - the wall bolts onto it, 4x M3 on Ø31 into nuts in pockets in the core (pushed in from the Ø16 cable bore),
the screws centring it; the rotor clamp's 4x M4 on Ø23 from counterbores in its front face (the heads sunk) through
both pieces into the shaft's nuts.

Designed here, no reference: tests/forearm/test_roll_drive.py holds its numbers (lib/forearm/roll.py
build_pulley(cfg), every number lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack
station - assemblies/forearm_roll_drive.py places it at 0). PETG, printed front face down (flat on the bed). In the arm: x1, inside
forearm_roll_drive#1 - forearm_link.
"""
import pathlib

from cad import step

from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_pulley

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: a pulley
FINE_LAYERS = True            # printed at lib/params.py LAYER_H_FINE: its GT2 teeth
REFERENCE = None
CONVERTED = True


def build(cfg: ForearmConfig = DEFAULT):
    return build_pulley(cfg)


@step(out="forearm_roll_pulley.step")
def forearm_roll_pulley():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_pulley()   # build: writes the sibling forearm_roll_pulley.step
