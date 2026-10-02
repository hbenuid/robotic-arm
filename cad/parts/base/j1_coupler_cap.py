"""j1_coupler_cap - the upper half of j1_coupler's motor-side leg, round the cycloidal drive's motor sleeve, PETG.

Designed here, no reference: tests/yaw_coupler/test_j1_coupler.py holds its numbers. DEFAULT's j1_coupler is a fork
round the drive whose shell turns (lib/yaw_coupler/params.py ForkParams): two alike legs past the shell's ends; the
motor-side one clamps the motor plate's sleeve, split at the drive's axis - its lower half is j1_coupler, this is the
upper half: the leg's plate_r disc above the axis, bored to the sleeve (clamp_bore_dia), its 2x M3 x cap_screw_len
through it from counterbores cap_seat above the split, into nuts in slots from the leg's sides (lib/yaw_coupler/body.py
build_cap, in j1_coupler's part frame - lib/mounts.py places it at identity on j1_coupler#1). The screws and nuts are on
the buy list (tools/bom.py EXTRAS).
"""
import pathlib

from cadgen import step

from lib.yaw_coupler import DEFAULT
from lib.yaw_coupler.body import build_cap

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


@step
def j1_coupler_cap():
    """The cap in j1_coupler's part frame (on the base_yaw axis at the seat-ring top); the assembly owns placement."""
    part = build_cap(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j1_coupler_cap()   # build: writes the sibling j1_coupler_cap.step
