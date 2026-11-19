"""The forearm tests' own helper (tests/forearm/test_*.py; the shared geometry helpers are tests/helpers.py): the
forearm's frames - every forearm part is modelled in j2_link's part frame, and in_host() brings any placed
occurrence of the arm into that frame for clearance checks; the roll drive is mounted in j2_link's frame BEFORE the
forearm's turn about the roll axis (lib/placements.py TURNS, lib/mounts.py ModuleMount), and in_drive_frame() /
j2_link_in_drive_frame() bring the arm and the turned forearm (j2_link with its wall bracket: forearm()) into that
one (the elbow axis and the module's frame are given in it: lib/forearm/layout.py module_frame_in_host)."""
from __future__ import annotations

from lib import placements as P
from tests import built, picking

HOST = "j2_link#1"


def in_host(key: str):
    """A part occurrence of placements.json, placed in j2_link's frame (tests/built.py: shared, read-only)."""
    return built.placed(key, into=HOST)


@picking.cache
def forearm():
    """j2_link and its wall bracket (printed apart, lib/forearm/params.py WallSplitParams; the bracket built in j2_link's
    frame) fused into the one forearm - the wall, its gussets, the web, the wrist boss - that the end's and the
    clearance tests probe (shared, read-only)."""
    return built.part("j2_link").fuse(built.part("j2_wall_bracket")).clean()


@picking.cache
def drive_frame_from_host():
    """j2_link's part frame (turned with the forearm) in its frame before the turn: half a turn about the roll axis."""
    return P.location(HOST, "world", turned=False).inverse() * P.location(HOST, "world")


@picking.cache
def in_drive_frame(key: str):
    """A part occurrence of placements.json, placed in j2_link's frame before the forearm's turn - the roll drive's
    mount frame (shared, read-only)."""
    return built.placed(key).moved(P.location(HOST, "world", turned=False).inverse())


@picking.cache
def j2_link_in_drive_frame():
    """j2_link with its wall bracket (forearm()) as the arm turns them, in the roll drive's mount frame (shared,
    read-only)."""
    return forearm().moved(drive_frame_from_host())
