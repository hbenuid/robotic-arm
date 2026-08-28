"""Build one rigid link of the robot as a labelled Compound in the LINK's own frame.

A link is the set of part occurrences that move together (robot/frames.py LINKS). Every
occurrence is placed at its SolidWorks world placement and re-expressed in the link frame
(= the frame of the joint whose child the link is, at the capture pose), so the exported
mesh needs an identity <origin> in the URDF.
"""
from __future__ import annotations

from assemblies._occurrences import place_world
from lib import placements as P
from lib.assembly import AssemblyHelper
from robot import frames as F


def build_link(link: str):
    keys = F.LINKS[link]
    frame = F.link_frame_world(link)
    counts: dict[str, int] = {}
    for key in keys:
        counts[P.OCCURRENCES[key]["part"]] = counts.get(P.OCCURRENCES[key]["part"], 0) + 1
    asm = AssemblyHelper(link)
    for key in keys:
        part = P.OCCURRENCES[key]["part"]
        shape = place_world(part, key, into=frame)
        if counts[part] > 1:
            asm.add(shape, part, key.rsplit("#", 1)[1])
        else:
            asm.add(shape, part)
    return asm.build()
