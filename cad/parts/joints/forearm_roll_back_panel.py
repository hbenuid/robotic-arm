"""forearm_roll_back_panel - the forearm roll frame's removable back panel: a plate flush with the frame's slanted rear
face (the rear line) in a step cut across the face's full width, from the tower's top down, over the shaft's bay and the
opening grown down below the bay's bore - lift it off and the roll shaft, its clamp nuts and the stop post are open to
reach. Relieved where it passes over the turning shaft (run_gap off its rear face and its lug), located by a U rim on its
inner face in the opening's lower part, held by 4 magnet pairs (roll_back_magnets: one disc in a pocket in the frame's
step, one in a pocket in its inner face). Nothing passes through it: the forearm's cables take another route.

Designed here, no reference: tests/forearm/test_roll_back_panel.py holds its numbers (lib/forearm/roll.py
build_back_panel(cfg), every number lib/forearm/params.py BackPanelParams, in the drive's MODULE frame at its station -
assemblies/forearm_roll_drive.py places it at 0). PETG, printed outer face down. In the arm: x1, inside
forearm_roll_drive#1 - elbow_link (the stator).
"""
import pathlib

from cadgen import step

from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_back_panel

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: ForearmConfig = DEFAULT):
    return build_back_panel(cfg)


@step(out="forearm_roll_back_panel.step")
def forearm_roll_back_panel():
    """The part in the module frame at its station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_back_panel()   # build: writes the sibling forearm_roll_back_panel.step
