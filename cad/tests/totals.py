"""What an occurrence of reference/placements.json contributes to the totals the geometry tests lock.

The SolidWorks records carry the solids / volume / world bbox of the SolidWorks export; the arm and link
tests sum them (tests/test_assembly.py, tests/test_robot.py). A CUSTOM part that has been CONVERTED to
build123d no longer IS that export - it matches it within tests/test_reference_match.py's tolerance
(0.5 % of volume), or deliberately diverges (REFERENCE_BUILD) - so for such an occurrence the totals come
from the part's own build, placed at the record's world pose. Everything else (wrappers, purchased parts,
the mounted motors) keeps the record's numbers.
"""
from __future__ import annotations

import parts
from lib import placements as P
from lib import reference as R
from tests import built


def is_converted(part: str) -> bool:
    """A SolidWorks-origin part whose geometry is now its own build123d model."""
    return part in R.CUSTOM and bool(parts.load(part).CONVERTED)


def solids_and_volume(key: str) -> tuple[int, float]:
    """(solids, solid_volume) of a part occurrence: the record's, or the built part's when converted."""
    o = P.OCCURRENCES[key]
    if is_converted(o["part"]):
        shape = built.part(o["part"])
        return len(shape.solids()), R.solid_volume(shape)
    return o["solids"], o["solid_volume"]


def world_bbox(key: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(min, size) of a part occurrence's world bounding box: the record's, or the placed build's when converted."""
    o = P.OCCURRENCES[key]
    if is_converted(o["part"]):
        bb = built.placed(key).bounding_box()
        return (bb.min.X, bb.min.Y, bb.min.Z), (bb.size.X, bb.size.Y, bb.size.Z)
    return tuple(o["world_bbox_min"]), tuple(o["world_bbox_size"])


def part_totals(keys) -> tuple[int, float]:
    """Summed (solids, solid_volume) over part occurrence keys."""
    pairs = [solids_and_volume(k) for k in keys]
    return sum(s for s, _ in pairs), sum(v for _, v in pairs)
