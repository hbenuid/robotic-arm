"""Build one rigid link of the robot as a labelled Compound in the LINK's own frame.

A link is the set of part occurrences that move together (robot/frames.py LINKS; a designed
module key such as "cycloidal_drive#1" contributes all of its parts, "cycloidal_drive#1:rotor"
one rigid body of it). Every occurrence is placed at its world placement and re-expressed in
the link frame (= the frame of the joint whose child the link is, at the capture pose), so the
exported mesh needs an identity <origin> in the URDF.
"""
from __future__ import annotations

from assemblies._occurrences import place_world_at, world_rows
from lib.assembly import assembly, label_shape
from robot import frames as F


def link_rows(link: str) -> list:
    """(key, part, role, world Location) for every part in the link - part keys give one row,
    designed-module keys (the cycloidal drive, or one of its ":<body>" rigid bodies) expand into
    their parts."""
    return [(key, part, role, world) for key in F.LINKS[link] for part, role, world in world_rows(key)]


def build_link(link: str):
    frame = F.link_frame_world(link)
    rows = link_rows(link)
    counts: dict[str, int] = {}
    for _, part, _, _ in rows:
        counts[part] = counts.get(part, 0) + 1
    children = []
    for key, part, role, world in rows:
        shape = place_world_at(part, world, into=frame)
        if role is not None:                      # module rows carry their own role (bearing_6003:1)
            children.append(label_shape(shape, part, role))
        elif counts[part] > 1:
            children.append(label_shape(shape, part, key.rsplit("#", 1)[1]))
        else:
            children.append(label_shape(shape, part))
    return assembly(link, children)
