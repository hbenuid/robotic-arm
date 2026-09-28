"""The forearm tests' own helper (tests/forearm/test_*.py; the shared geometry helpers are tests/helpers.py): the
forearm's frame - every forearm part is modelled in j2_link's part frame, and in_host() brings any placed
occurrence of the arm into that frame for clearance checks."""
from __future__ import annotations

from tests import built

HOST = "j2_link#1"


def in_host(key: str):
    """A part occurrence of placements.json, placed in j2_link's frame (tests/built.py: shared, read-only)."""
    return built.placed(key, into=HOST)
