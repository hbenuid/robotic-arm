"""The forearm tests' own helper (tests/forearm/test_*.py; the shared geometry helpers are tests/helpers.py): the
forearm's frame - every forearm part is modelled in j2_link's part frame, and in_host() brings any placed
occurrence of the arm into that frame for clearance checks."""
from __future__ import annotations

from build123d import Location

from assemblies._occurrences import place_world
from lib import placements as P

HOST = "j2_link#1"


def host_world() -> Location:
    return P.location(HOST, "world")


def in_host(key: str):
    """A part occurrence of placements.json, placed in j2_link's frame."""
    return place_world(P.OCCURRENCES[key]["part"], key, into=host_world())
