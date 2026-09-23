"""The LEGACY forearm configuration reproduces the three SolidWorks parts feature by feature (the
volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a
regression names what moved)."""
import pytest

from lib.forearm import LEGACY, cap1_socket_points, cap2_socket_points, link_socket_points, motor_window
from tests.forearm.helpers import is_inside


def test_legacy_layout():
    assert motor_window(LEGACY) == (-148.0, -88.0, 24.0)
    top, bottom = link_socket_points(LEGACY)
    assert len(top) == 8 and len(bottom) == 10
    assert len(cap1_socket_points(LEGACY)) == 8
    assert len(cap2_socket_points(LEGACY)) == 8 and (-30.0, 40.0) not in cap2_socket_points(LEGACY)


@pytest.fixture(scope="module")
def link():
    from lib.forearm.link import build_link
    return build_link(LEGACY)


@pytest.fixture(scope="module")
def cap1():
    from lib.forearm.caps import build_cap_1
    return build_cap_1(LEGACY)


@pytest.fixture(scope="module")
def cap2():
    from lib.forearm.caps import build_cap_2
    return build_cap_2(LEGACY)


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


@pytest.mark.slow
def test_cap_1_features(cap1):
    assert cap1.is_valid and len(cap1.solids()) == 1
    assert is_inside(cap1, -60, 0, 33) and not is_inside(cap1, -60, 0, 25)      # lid over the pocket
    assert is_inside(cap1, -100, 40, 25) and not is_inside(cap1, -100, 30, 25)  # rim / pocket wall at y 35
    assert not is_inside(cap1, 20, 0, 25) and is_inside(cap1, 40, 0, 25)        # the pocket wraps the elbow axis
    assert is_inside(cap1, -160, 0, 25) and not is_inside(cap1, -170, 0, 25)    # rim beyond the r55 arc, nothing inside r46
    assert not is_inside(cap1, -118, 0, 32.75) and is_inside(cap1, -80, 0, 32.75)   # the window
    assert not is_inside(cap1, -120, 40, 20) and is_inside(cap1, -120, 40, 22)     # a socket


@pytest.mark.slow
def test_cap_2_features(cap2):
    assert cap2.is_valid and len(cap2.solids()) == 1
    assert is_inside(cap2, -150, 0, -4.75) and not is_inside(cap2, -150, 0, 2)  # floor under the pocket
    assert is_inside(cap2, -150, 40, 2) and not is_inside(cap2, -150, 30, 2)    # rim / pocket wall
    assert not is_inside(cap2, -60, 0, 2) and is_inside(cap2, -62, 0, -4.75)    # open toward the elbow, floor lip
    assert not is_inside(cap2, -80, 0, -4.75) and not is_inside(cap2, -80, 40, 2)   # the channel, floor and rim
    assert is_inside(cap2, -80, 40, 7)                                              # ... but not the rim face
    assert not is_inside(cap2, -235, 0, 2) and is_inside(cap2, -250, 0, 2)      # the pocket wraps the wrist axis
    assert not is_inside(cap2, -120, 40, 7) and is_inside(cap2, -120, 40, 5)    # a socket

