"""j1_motor_leg - j1_coupler's motor-side leg, its own part: a solid ring round the cycloidal drive's motor sleeve,
bolted to the coupler's disc and to the sleeve, PETG.

Designed here, no reference: tests/yaw_coupler/test_j1_coupler.py holds its numbers. DEFAULT's j1_coupler is a fork
round the drive whose shell turns (lib/yaw_coupler/params.py ForkParams): two legs past the shell's ends, the disc's
drafted side carried up their outer faces. The hub-side leg is j1_coupler's; this is the other - a solid ring (no split,
no cap) round the motor plate's sleeve (ring_bore_dia), slid on over the motor and its board once the drive is in, on a
post standing on the disc's top face; its foot the disc's rim past the leg's outer face, butting the disc there, held by
2x M4 along the drive's axis from counterbores in the foot into nuts in slots in the disc, under the post; and the
sleeve held in it - nothing else stops the drive's carrier turning at the motor end - by 3 radial M4 x 20 through the
ring's middle (up and both sides: the motor's flats) into nuts pressed into lugs on the sleeve's bore, the holes
slotted along the axis so the drive settles lengthwise before they lock it (ForkParams.sleeve_screw_len /
sleeve_slot_len; lib/yaw_coupler/body.py build_motor_leg, in j1_coupler's part frame - lib/mounts.py places it at
identity on j1_coupler#1). The screws and nuts are purchased parts of their own, mounted on it (fork_foot_screws /
fork_foot_nuts, fork_sleeve_screws / fork_sleeve_nuts: lib/mounts.py FORK_MOUNTS).
"""
import pathlib

from cadgen import step

from lib.yaw_coupler import DEFAULT
from lib.yaw_coupler.body import build_motor_leg

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


@step(out="j1_motor_leg.step")
def j1_motor_leg():
    """The motor leg in j1_coupler's part frame (on the base_yaw axis at the seat-ring top); the assembly owns placement."""
    part = build_motor_leg(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j1_motor_leg()   # build: writes the sibling j1_motor_leg.step
