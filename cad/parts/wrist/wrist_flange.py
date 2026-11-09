"""wrist_flange - the wrist_pitch output flange, its own part on wrist_link's coupler face about the pitch axis: the Ø40
post (the SolidWorks j3_coupler's journal carried down, cut flat at wrist_link's front face and rear, its foot toward
the bearing chamfered), the Ø33 shoulder on the upper wrist 6806's inner ring and the Ø30 stub down through it and
j2_link's lip to the wrist 90T, which bolts flat onto its end; the Ø12.5 pin bore, blind from the stub's end; the 90T's
4x M4 clearance through it; the octagonal recess in its foot over wrist_link's key, which centres it and keys it against
turning.

Its own part so that its round faces print as circles: printed on the stub's end, where on wrist_link - printed with
its bearing seats upright - they lay on their side (each printed part keeps its round features about one axis). The
wrist 90T's 4x M4 x pulley_screw_len (wrist_pulley_screws) run through the 90T and the flange into the nuts in
wrist_link's plate (wrist_pulley_nuts): the 90T, the flange and wrist_link one clamp.

Designed here, no reference: tests/wrist/test_wrist_flange.py holds its numbers (lib/wrist/link.py
build_wrist_flange(cfg), every number lib/wrist/params.py FlangeParams, in wrist_link's part frame - lib/mounts.py
places it on wrist_link#1 as it stands). PETG. In the arm: x1 (wrist_flange#1), wrist_pitch_link.
"""
import pathlib

from cad import step

from lib.wrist import DEFAULT, WristConfig
from lib.wrist.link import build_wrist_flange

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


def build(cfg: WristConfig = DEFAULT):
    return build_wrist_flange(cfg)


@step(out="wrist_flange.step")
def wrist_flange():
    """The part in wrist_link's part frame (the mount places it on wrist_link as it stands)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_flange()   # build: writes the sibling wrist_flange.step
