"""elbow_flange - the elbow's output flange, the forearm roll drive's stator's own part under the frame's underside about
the elbow axis: a Ø40 post (the SolidWorks j3_coupler's journal carried up, its foot chamfered), the Ø33 shoulder on the
upper elbow 6806's inner ring and the Ø30 stub down through it and j1_link's lip to the elbow 90T, which bolts flat onto
its end; the Ø12.5 pin bore, blind from the stub's end; the 90T's 4x M4 clearance through it; the octagonal recess in
its top over the frame's spigot (forearm_roll_block), which centres it and keys it against turning.

Its own part so that its round faces print as circles: printed on the stub's end (the shoulder and the post's foot
step out over it), where on the frame - printed front face down for its 6806 seats - they lay on their side. The elbow
90T's 4x M4 x pulley_screw_len (elbow_pulley_screws) run through the 90T, the flange and the spigot into the nuts in the
frame (elbow_pulley_nuts): the 90T, the flange and the frame one clamp.

Designed here, no reference: tests/forearm/test_elbow_flange.py holds its numbers (lib/forearm/roll.py
build_elbow_flange(cfg), every number lib/forearm/params.py RollDriveParams post_* / step_* / stub_* / spigot_*, in the
drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py places it at 0). PETG. In the arm: x1,
inside forearm_roll_drive#1 - elbow_link (the stator).
"""
import pathlib

from cadgen import step

from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_elbow_flange

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: ForearmConfig = DEFAULT):
    return build_elbow_flange(cfg)


@step(out="elbow_flange.step")
def elbow_flange():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    elbow_flange()   # build: writes the sibling elbow_flange.step
