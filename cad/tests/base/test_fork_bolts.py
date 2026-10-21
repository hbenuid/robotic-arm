"""fork_{hub,foot,sleeve}_{screws,nuts}, the M4 screws and nuts that hold the cycloidal drive in j1_coupler's fork - the
hub's 4 to the hub leg, the motor leg's foot's 2 to the disc, its ring's 3 to the motor plate's sleeve: purchased parts
with no reference (lib/reference.py NO_REFERENCE), which keep no reference file - THIS file locks their numbers (the
pieces, the modelled volume and the bounding box, measured on the build and written in) and what they are: every screw
on its pattern with its head's bearing face at its origin, every nut there with a corner where its pocket or slot has
one (lib/yaw_coupler/layout.py fork_*_screw_*).
The patterns in place - the heads on their seats, the nuts on theirs with the designed press, the reach - are
tests/test_mounts.py."""
import math

import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.cycloidal import arm_mount_points
from lib.geom import hex_circumdiameter
from lib.yaw_coupler import DEFAULT as YAW_COUPLER
from lib.yaw_coupler.layout import (
    fork_foot_screw_points,
    fork_hub_screw_points,
    fork_sleeve_bolt_deg,
    fork_sleeve_nut_r,
)
from tests import built
from tests.helpers import is_inside

F = YAW_COUPLER.fork
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "fork_hub_screws": (4, 1810.035, (-21.17767, -21.17767, -4.0), (42.355339, 42.355339, 29.0)),
    "fork_hub_nuts": (4, 382.322, (-21.581412, -21.581412, 0.0), (43.162825, 43.162825, 3.2)),
    "fork_foot_screws": (2, 528.027, (-15.5, -3.5, -4.0), (31.0, 7.0, 14.0)),
    "fork_foot_nuts": (2, 191.161, (-15.5, -4.041452, 0.0), (31.0, 8.082904, 3.2)),
    "fork_sleeve_screws": (3, 1169.031, (-49.0, -3.5, -3.5), (98.0, 52.5, 7.0)),
    "fork_sleeve_nuts": (3, 286.741, (-28.2, -4.041452, -3.5), (56.4, 32.241452, 7.0)),
}
# each set: (its screws' length, its mass constant, its nuts' mass constant)
SETS = {
    "hub": (F.hub_screw_len, PARAMS.FORK_HUB_SCREWS_MASS_G, PARAMS.FORK_HUB_NUTS_MASS_G),
    "foot": (F.motor_screw_len, PARAMS.FORK_FOOT_SCREWS_MASS_G, PARAMS.FORK_FOOT_NUTS_MASS_G),
    "sleeve": (F.sleeve_screw_len, PARAMS.FORK_SLEEVE_SCREWS_MASS_G, PARAMS.FORK_SLEEVE_NUTS_MASS_G),
}


def _unit(v) -> tuple[float, float, float]:
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def _cross(a, b) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _stations(name: str) -> list[tuple[tuple, tuple, tuple]]:
    """(origin, axis, corner) of each piece in its part frame: the head's or the nut's bearing face's centre, the screw's
    direction (the nut's from its bearing face), the direction of a corner of the nut's hex."""
    up, kind = (0.0, 0.0, 1.0), name.split("_")[1]
    if kind == "hub":
        return [((x, y, 0.0), up, _unit((x, y, 0.0))) for x, y in fork_hub_screw_points()]
    if kind == "foot":
        return [((x, y, 0.0), up, (0.0, 1.0, 0.0)) for x, y in fork_foot_screw_points()]
    r = F.plate_r if name.endswith("screws") else fork_sleeve_nut_r()
    out = []
    for deg in fork_sleeve_bolt_deg():
        u = (math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0)
        out.append(((r * u[0], r * u[1], 0.0), (-u[0], -u[1], 0.0), (-u[1], u[0], 0.0)))
    return out


def _at(origin, axis, side, along: float, out: float = 0.0, across=None):
    """The point `along` the axis from origin and `out` from it toward `side` (or along `across`)."""
    d = side if across is None else across
    return tuple(o + along * a + out * s for o, a, s in zip(origin, axis, d, strict=True))


def _inside(shape, p) -> bool:
    return any(is_inside(s, *p) for s in shape.solids())


def test_they_are_purchased_parts_with_no_reference_in_the_base_group():
    for name in LOCKS:
        mod = parts.load(name)
        assert parts.GROUPS[name] == "base" and parts.bought(name) and mod.PURCHASE_QTY == LOCKS[name][0]
        assert R.NO_REFERENCE[name] == f"parts/base/{name}.py:_envelope()"
    assert LOCKS["fork_hub_screws"][0] == PARAMS.CYCLOIDAL_HUB_BOLT_COUNT == len(arm_mount_points(DRIVE)) == 4
    assert LOCKS["fork_sleeve_screws"][0] == PARAMS.CYCLOIDAL_SLEEVE_BOLT_COUNT == len(fork_sleeve_bolt_deg()) == 3
    assert LOCKS["fork_foot_screws"][0] == len(fork_foot_screw_points()) == 2


@pytest.mark.parametrize("kind", list(SETS))
def test_the_order_lines_and_masses_follow_the_fork(kind):
    length, screws_g, nuts_g = SETS[kind]
    screws, nuts = parts.load(f"fork_{kind}_screws"), parts.load(f"fork_{kind}_nuts")
    assert screws.LENGTH == length and screws.PURCHASE_SPEC == f"M4 x {length:g} socket head cap screw (ISO 4762)"
    assert nuts.PURCHASE_SPEC == "M4 hex nut (ISO 4032)" and nuts.PURCHASE_QTY == screws.PURCHASE_QTY
    steel = PARAMS.STEEL_DENSITY * screws.PURCHASE_QTY
    assert screws.MASS_G == screws_g == pytest.approx(steel * PARAMS.shcs_volume(PARAMS.M4_SHCS, length))
    assert nuts.MASS_G == nuts_g == pytest.approx(steel * PARAMS.nut_volume(PARAMS.M4_NUT))
    assert (F.hub_screw_len, F.motor_screw_len, F.sleeve_screw_len) == (25.0, 10.0, 20.0)


@pytest.mark.slow
@pytest.mark.parametrize("name", list(LOCKS))
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)


@pytest.mark.slow
@pytest.mark.parametrize("kind", list(SETS))
def test_the_modelled_volumes_are_the_analytic_ones(kind):
    """lib/fasteners.py's volumes (what the masses are estimated from) are those of the modelled geometry."""
    n, length = LOCKS[f"fork_{kind}_screws"][0], SETS[kind][0]
    assert R.solid_volume(built.part(f"fork_{kind}_screws")) == pytest.approx(n * PARAMS.shcs_volume(PARAMS.M4_SHCS, length), rel=1e-6)
    assert R.solid_volume(built.part(f"fork_{kind}_nuts")) == pytest.approx(n * PARAMS.nut_volume(PARAMS.M4_NUT), rel=1e-6)


@pytest.mark.slow
@pytest.mark.parametrize("kind", list(SETS))
def test_each_screw_stands_on_its_station(kind):
    """The head behind its bearing face (its socket open at the back), the shank along the axis to the length: the hub's
    and the foot's along +Z on their points, the sleeve's pointing in at the axis from the ring's od."""
    name, size, length = f"fork_{kind}_screws", PARAMS.M4_SHCS, SETS[kind][0]
    screws = built.part(name)
    for origin, axis, side in _stations(name):
        assert _inside(screws, _at(origin, axis, side, length - 0.1)) and not _inside(screws, _at(origin, axis, side, length + 0.1))
        assert _inside(screws, _at(origin, axis, side, length / 2.0, size.d / 2.0 - 0.1))
        assert not _inside(screws, _at(origin, axis, side, length / 2.0, size.d / 2.0 + 0.1))
        assert _inside(screws, _at(origin, axis, side, -size.head_h + 0.1, size.head_dia / 2.0 - 0.1))   # the head's rim, to its back
        assert not _inside(screws, _at(origin, axis, side, -size.head_h + size.socket_depth - 0.1))       # the socket
        assert _inside(screws, _at(origin, axis, side, -size.head_h + size.socket_depth + 0.1))


@pytest.mark.slow
@pytest.mark.parametrize("kind", list(SETS))
def test_each_nut_has_a_corner_where_its_seat_has_one(kind):
    """Standing on its bearing face, bored at the thread's diameter, a corner (and a flat 30 degrees on) where the hub's
    pockets (along the radius), the disc's slots (up them, CORNER_DEG) and the sleeve's lugs (along the tangent) have one
    (tests/test_mounts.py checks the seats and the press)."""
    name, size = f"fork_{kind}_nuts", PARAMS.M4_NUT
    nuts, corner_r = built.part(name), hex_circumdiameter(size.af) / 2.0 - 0.05
    if kind == "foot":
        assert parts.load(name).CORNER_DEG == 90.0
    for origin, axis, corner in _stations(name):
        other = _cross(axis, corner)
        assert not _inside(nuts, _at(origin, axis, corner, size.h / 2.0))                                   # the bore
        assert not _inside(nuts, _at(origin, axis, corner, size.h / 2.0, size.d / 2.0 - 0.1))
        assert _inside(nuts, _at(origin, axis, corner, size.h / 2.0, size.d / 2.0 + 0.1))
        for k in range(6):
            a = k * math.pi / 3.0
            d = tuple(math.cos(a) * c + math.sin(a) * o for c, o in zip(corner, other, strict=True))
            assert _inside(nuts, _at(origin, axis, corner, size.h / 2.0, corner_r, d)), (origin, k)
            f = tuple(math.cos(a + math.pi / 6.0) * c + math.sin(a + math.pi / 6.0) * o for c, o in zip(corner, other, strict=True))
            assert not _inside(nuts, _at(origin, axis, corner, size.h / 2.0, corner_r, f)), (origin, k)
        flat = tuple(math.cos(math.pi / 6.0) * c + math.sin(math.pi / 6.0) * o for c, o in zip(corner, other, strict=True))
        assert not _inside(nuts, _at(origin, axis, corner, size.h + 0.1, size.af / 2.0 - 0.1, flat))
        assert not _inside(nuts, _at(origin, axis, corner, -0.1, size.af / 2.0 - 0.1, flat))
        assert _inside(nuts, _at(origin, axis, corner, size.h / 2.0, size.af / 2.0 - 0.1, flat))
