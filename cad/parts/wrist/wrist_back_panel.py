"""wrist_back_panel - the wrist frame's removable back panel (the forearm roll frame's, one joint further out): a plate
flush with wrist_link's slanted rear over the tower, in a step, over the shaft's bay bored through the rear - lift it off
and the wrist roll shaft, its clamp nuts and the stop post are open to reach, and the shaft goes in or out along the
roll axis. Located by a ring rim on its inner face inside the bay's bore, held by 4 magnet pairs (wrist_back_magnets:
one disc in a pocket in the frame's step, one in a pocket in its inner face); a notch at its edge toward the motor lets
the gripper servo's cable out of the rotor's bore, another clears the MKS board's near side plug. A grill of straight
slots is cut through it over the bay, inside the rim - the turning shaft shows behind it (lib/wrist/roll.py back_grill).

Designed here, no reference: tests/wrist/test_back_panel.py holds its numbers (lib/wrist/roll.py build_back_panel(cfg),
every number lib/wrist/params.py WristBackPanelParams, in the drive's MODULE frame at its station -
assemblies/wrist_roll_drive.py places it at 0). PETG, printed outer face down. In the arm: x1, inside wrist_roll_drive#1 -
wrist_pitch_link (the stator).
"""
import pathlib

from cadgen import step

from lib.wrist import DEFAULT, WristConfig
from lib.wrist.roll import build_back_panel

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: a cover
REFERENCE = None
CONVERTED = True


def build(cfg: WristConfig = DEFAULT):
    return build_back_panel(cfg)


@step(out="wrist_back_panel.step")
def wrist_back_panel():
    """The part in the module frame at its station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_back_panel()   # build: writes the sibling wrist_back_panel.step
