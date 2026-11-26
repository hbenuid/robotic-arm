"""j2_wall_bracket - the forearm's elbow end, PETG: the roll wall the forearm roll pulley bolts onto, its two gussets
and the tongue that laps onto j2_link's web.

Designed here, no reference: tests/forearm/test_wall_split.py holds its numbers. DEFAULT's forearm prints in two parts
(lib/forearm/params.py WallSplitParams): whole, the wall stands upright on the web while the wrist boss lies flat on
it, so one of their round features always printed on its side. This is the wall's part - the faceted flange about the
roll axis, its elbow face flat on the roll pulley's flat front face, the rotor clamp's 4 M4 holes (the screws' heads in counterbores in its
wrist face: roll_clamp_screws, on through the pulley and the shaft) and the cable bore; the two gussets; the tongue, the
web's top half from the wall's wrist face to WallSplitParams.lap_x, with the lap screws' holes and their nuts' hex
pockets in its top face (parts/joints/wall_bracket_nuts). It prints on the wall's elbow face: the flange's holes
stand upright, the tongue and the gussets up from it. In j2_link's part frame (lib/forearm/link.py
build_wall_bracket); lib/mounts.py places it at identity on j2_link#1.
"""
import pathlib

from cad import step

from lib.forearm import DEFAULT
from lib.forearm.link import build_wall_bracket

NAME = pathlib.Path(__file__).stem
REFERENCE = None
CONVERTED = True


@step(out="j2_wall_bracket.step")
def j2_wall_bracket():
    """The wall bracket in j2_link's part frame (the elbow pivot at the origin); the assembly owns placement."""
    part = build_wall_bracket(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j2_wall_bracket()   # build: writes the sibling j2_wall_bracket.step
