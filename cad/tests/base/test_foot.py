"""The base's leaning walls and its foot (lib/base/params.py ShellParams.draft, FootParams; lib/base/body.py): the walls
are j1_coupler's cone carried on down - flush under its rim, out at BASE_DRAFT going down -, they stand on a Ø200
flange with a straight chamfer up the wall, and 6 screw holes go down through the flange, their spot-faces clear of the
wall and of the joint face. The build's volume and box are measured on it and written in (tests/test_reference_match.py
matches the LEGACY build, which neither leans nor has a foot)."""
import math

import pytest

from lib.base import DEFAULT, LEGACY, flare_points, foot_holes, inner_r, mount_inner_half, outer_r
from lib.base.params import BASE_DRAFT, BASE_R
from lib.reference import solid_volume
from lib.yaw_coupler import DEFAULT as COUPLER
from lib.yaw_coupler.layout import disc_r
from lib.yaw_coupler.params import RIM_CLEAR
from tests import built
from tests.helpers import is_inside

S, C, P, F, J = DEFAULT.shell, DEFAULT.cap, DEFAULT.plate, DEFAULT.foot, DEFAULT.joint
VOLUME = 413546.1          # mm^3, measured on the build (the SolidWorks base: 286169.1, cut at the joint 235439.1)


def test_legacy_stands_straight_without_a_foot():
    assert LEGACY.shell.draft == 0.0 and LEGACY.foot is None
    assert outer_r(LEGACY, LEGACY.shell.y0) == outer_r(LEGACY, LEGACY.cap.top_y) == LEGACY.shell.r


def test_the_walls_are_the_couplers_cone_carried_down():
    """j1_coupler's frame origin is the base's ring_top_y (placements.json): along its drafted side, from the rim up,
    the coupler's radius is the base's cone; the rim RIM_CLEAR over the base's top face."""
    d = COUPLER.disc
    assert S.draft == BASE_DRAFT and outer_r(DEFAULT, C.ring_top_y) == pytest.approx(BASE_R)
    for y in (d.y0, (d.y0 + d.top_y) / 2.0, d.top_y):
        assert outer_r(DEFAULT, C.ring_top_y + y) == pytest.approx(disc_r(COUPLER, y), abs=1e-6)
    assert C.ring_top_y + d.y0 - C.top_y == pytest.approx(RIM_CLEAR)
    assert S.r == pytest.approx(outer_r(DEFAULT, C.top_y)) and S.r > LEGACY.shell.r     # flush, where it stood 4.7 in
    assert BASE_DRAFT == pytest.approx(math.tan(math.radians(10.8)), abs=1e-3)


def test_the_foot_and_its_chamfer():
    """The flange Ø200, a flat rim outside the chamfer; the chamfer from the flange's top to the wall, ~40 degrees, its
    top under the plate (the posts and the motor mount above it)."""
    (rf, yt), (rw, yw) = flare_points()
    assert F.r == 100.0 and yt == S.y0 + F.t
    assert F.r - rf == pytest.approx(4.5) and rf > outer_r(DEFAULT, yt) + 15.0
    assert rw == pytest.approx(outer_r(DEFAULT, yw)) and yw == S.y0 + F.flare_h < P.y[0] - 10.0
    assert math.degrees(math.atan2(yw - yt, rf - rw)) == pytest.approx(39.66, abs=0.01)


def test_the_screw_holes():
    """6 holes on hole_r: the round half's 4 and one each side; each spot-face clear of the wall's outside at the bottom
    (the widest), inside the flange's rim, the side ones clear of the joint face; neighbours a spot-face apart."""
    holes = foot_holes()
    spot = F.spot_dia / 2.0
    assert len(holes) == 6 and F.hole_dia < F.spot_dia
    for x, z in holes:
        reach = math.hypot(x, z) if x < 0.0 else abs(z)        # from the axis (round half) or the side's plane
        assert reach == pytest.approx(F.hole_r)
        assert reach - spot - outer_r(DEFAULT, S.y0) > 8.0 and reach + spot < F.r
    sides = [(x, z) for x, z in holes if x > 0.0]
    assert len(sides) == 2 and all(x + spot < J.split_x - 1.0 for x, _ in sides)
    ring = sorted(holes, key=lambda h: math.atan2(h[1], h[0]))
    gaps = [math.dist(a, b) for a, b in zip(ring, ring[1:] + ring[:1], strict=True)]
    assert min(gaps) > 2.0 * F.spot_dia


@pytest.fixture(scope="module")
def base():
    return built.part("base")


@pytest.mark.slow
def test_the_build(base):
    assert base.is_valid and len(base.solids()) == 1
    assert solid_volume(base) == pytest.approx(VOLUME, abs=0.5)
    bb = base.bounding_box()
    assert tuple(bb.min) == pytest.approx((-F.r, S.y0, -F.r), abs=1e-6)
    assert tuple(bb.max) == pytest.approx((J.split_x, C.ring_top_y, F.r), abs=1e-6)


@pytest.mark.slow
def test_the_walls_lean_inside_and_out(base):
    """On the round half (-X, z = 0) and on a side (z +): the wall's outside on the cone, its inside `wall` in, below
    the plate and above it (under the cap's chamfer); the top face's edge flush under the coupler's rim; the cap a
    full disc, the walls open over the plate."""
    for y in (S.y0 + F.flare_h + 2.0, -70.0, P.y[0] - 1.0, -30.0):
        ro, ri = outer_r(DEFAULT, y), inner_r(DEFAULT, y)
        assert is_inside(base, -(ro - 0.3), y, 0.0) and not is_inside(base, -(ro + 0.3), y, 0.0), y
        assert is_inside(base, -(ri + 0.3), y, 0.0) and not is_inside(base, -(ri - 0.3), y, 0.0), y
    for y in (S.y0 + F.flare_h + 2.0, -70.0):
        ro = outer_r(DEFAULT, y)
        assert is_inside(base, 10.0, y, ro - 0.3) and not is_inside(base, 10.0, y, ro + 0.3), y
    assert is_inside(base, -(S.r - 0.3), C.top_y - 0.3, 0.0) and not is_inside(base, -(S.r + 0.3), C.top_y - 0.3, 0.0)
    # the cap a full disc over the open +X side (the leaning _d keeps _d's cylinder): the groove's floor, the top face
    assert is_inside(base, 40.0, C.groove_y0 - 0.5, 0.0) and is_inside(base, 50.0, C.top_y - 0.3, 0.0)
    assert not is_inside(base, 40.0, P.y[1] + 5.0, 0.0)                                       # open over the plate


@pytest.mark.slow
def test_the_foot_is_built(base):
    """The flange to Ø200 under the chamfer, the chamfer's face, the holes through and their spot-faces down to the
    flange's top; the posts out to the leaning wall, never through it."""
    yt = S.y0 + F.t
    assert is_inside(base, -(F.r - 0.5), yt - 0.5, 0.0) and not is_inside(base, -(F.r - 0.5), yt + 0.5, 0.0)
    (rf, _), (rw, yw) = flare_points()
    n = math.hypot(yw - yt, rf - rw)
    nr, ny = (yw - yt) / n, (rf - rw) / n                     # the chamfer's outward normal (out and up)
    mr, my = (rf + rw) / 2.0, (yt + yw) / 2.0
    assert is_inside(base, -(mr - 0.3 * nr), my - 0.3 * ny, 0.0) and not is_inside(base, -(mr + 0.3 * nr), my + 0.3 * ny, 0.0)
    spot = F.spot_dia / 2.0
    for x, z in foot_holes():
        ux, uz = (-x / F.hole_r, -z / F.hole_r) if x < 0.0 else (0.0, -math.copysign(1.0, z))   # toward the wall
        assert not is_inside(base, x, S.y0 + F.t / 2.0, z)                                   # the hole
        assert not is_inside(base, x + (spot - 0.5) * ux, yt + 1.0, z + (spot - 0.5) * uz)    # the spot-face ...
        assert is_inside(base, x + (spot + 0.5) * ux, yt + 1.0, z + (spot + 0.5) * uz)        # ... in the chamfer
        assert is_inside(base, x + (spot + 0.5) * ux, yt - 0.5, z + (spot + 0.5) * uz)        # the flange under it
    x = J.split_x - 1.0
    assert is_inside(base, x, -80.0, mount_inner_half() + 2.0)                                # a post
    assert not is_inside(base, x, P.y[0] - 2.0, outer_r(DEFAULT, P.y[0] - 2.0) + 0.3)       # inside the wall
