"""The LEGACY forearm configuration reproduces the SolidWorks j2_link feature by feature (the volume / bbox
match is tests/test_reference_match.py's; here the features are probed by name so a regression names what moved)."""
import pytest

from lib.forearm import LEGACY, link_socket_points
from tests.forearm.helpers import is_inside


def test_legacy_layout():
    top, bottom = link_socket_points(LEGACY)
    assert len(top) == 8 and len(bottom) == 10


@pytest.fixture(scope="module")
def link():
    from lib.forearm.link import build_link
    return build_link(LEGACY)


@pytest.mark.slow
def test_link_features(link):
    assert link.is_valid and len(link.solids()) == 1
    assert is_inside(link, -100, 30, 13.5)              # the web
    assert not is_inside(link, -100, 0, 13.5)           # the central slot
    assert not is_inside(link, -100, 15.4, 13.5)        # a side slot
    assert is_inside(link, -100, 12, 13.5)              # between them
    assert is_inside(link, 40, 0, 4) and not is_inside(link, 20, 0, 4)     # the disc and its bore
    assert not is_inside(link, 38, 0, 17.5) and is_inside(link, 39.5, 0, 17.5)   # the hex pocket (a vertex toward the centre) ...
    assert not is_inside(link, 35, 0, 10) and is_inside(link, 38, 0, 10)         # ... over the narrower M4 hole
    assert is_inside(link, -190, 0, 18) and not is_inside(link, -190, 0, 12)   # the lip inside the seat
    assert not is_inside(link, -175, 0, 30) and is_inside(link, -168, 0, 30)   # the recess and the boss wall
    assert not is_inside(link, -120, 40, 18) and not is_inside(link, -120, 40, 9) and is_inside(link, -120, 40, 13.5)   # sockets
    assert not is_inside(link, -244.64, 20, 9)          # the wrist-end socket
