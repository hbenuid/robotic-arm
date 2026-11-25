"""The forearm printed in two parts (lib/forearm/params.py WallSplitParams): whole, the roll wall stands upright on the web
while the wrist boss lies flat on it, so one of their round features always printed on its side. j2_wall_bracket (the
wall, its two gussets, the tongue - the web's top half from the wall's wrist face to lap_x) prints on the wall's elbow
face, j2_link (the web, its end stepped down under the tongue) on its underside. Together they are the whole forearm
(lib/forearm/link.py _build_whole) less the lap's holes; the lap is held by 3 M3 up from the web's underside
(wall_bracket_screws, the heads flush in counterbores) into nuts in hex pockets in the tongue's top face
(wall_bracket_nuts, flush) - and by the wrist motor's two front screws, which pass through it. The bracket and the
fasteners have no reference (lib/reference.py NO_REFERENCE), so THIS file locks their numbers (pieces, the modelled
volume and the bounding box, measured on the build and written in)."""
import math

import pytest
from cad.build123d import closest_points

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import (
    DEFAULT,
    LEGACY,
    flange_bolt_points,
    lap_nut_z,
    lap_screw_head_z,
    lap_z,
    motor_hole_points,
    stack_positions,
)
from lib.forearm.link import _build_whole
from lib.geom import hex_circumdiameter
from lib.style import SIDES
from tests import built
from tests.helpers import interference, is_inside

S, W, R_END, H = DEFAULT.split, DEFAULT.web, DEFAULT.roll_end, DEFAULT.holes
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "j2_wall_bracket": (1, 27925.812, (-86.0, -33.556206, -5.0), (38.0, 67.112413, 60.0)),
    "wall_bracket_screws": (3, 362.361, (-69.75, -10.75, -3.0), (11.5, 21.5, 11.0)),
    "wall_bracket_nuts": (3, 137.727, (-70.175426, -10.75, 0.0), (12.350853, 21.5, 2.4)),
}
J2_LINK_VOLUME = 88252.097   # mm^3, the web part (the whole forearm less the bracket and the lap's holes)


def test_the_configs():
    assert LEGACY.split is None and DEFAULT.split is not None


def test_the_wall_and_the_pulley_meet_flat():
    """The bracket prints on the wall's elbow face and the roll pulley on its front face: both flat (no spigot, no
    recess - its floor would print as a bridge), the pulley's front face the ring's, on the wall (stack_positions);
    the wall's screws centre it."""
    S_ = stack_positions()
    assert "z_spigot_end" not in S_ and S_["z_ring_end"] == S_["z_wall"] == -R_END.wall_x[1]
    assert not any(hasattr(R_END, name) for name in ("flange_dia", "flange_recess_depth", "flange_recess_clear"))


def test_the_lap_stations():
    """Along N: the lap on the web's mid-plane; the heads' bearing face head_h over the underside (flush), the tips
    flush with the tongue's top face, through the whole nut, which sits flush in the top face, the tongue's 2.9 under it.
    Along the web: the tongue from the wall's wrist face past the wrist motor's front bolt holes, short of its pilot
    hole. Across: every screw where the web is whole at the wall (inside the corner facets' feet), off the wall's
    bottom screw's head, the motor's bolt holes and its pilot."""
    assert lap_z() == pytest.approx((W.z0 + W.z1) / 2.0) == pytest.approx(0.5)
    assert lap_screw_head_z() == pytest.approx(W.z0 + S.screw.head_h)
    assert lap_screw_head_z() + S.screw_len == pytest.approx(W.z1)                       # the tips flush with the top face
    assert lap_nut_z() == pytest.approx(W.z1 - S.nut.h) and lap_nut_z() - lap_z() >= 2.5
    assert S.head_dia == H.pocket_dia and S.bolt_dia == H.bolt_dia
    front_holes = max(x for x, _ in motor_hole_points()) - H.bolt_dia / 2.0
    pilot = DEFAULT.motor_x + H.pilot_dia / 2.0
    assert front_holes - S.lap_x == pytest.approx(S.lap_x - pilot, abs=0.1) and S.lap_x - pilot >= 1.2   # midway, 1.3 either side
    corner_y = R_END.wall_od / 2.0 * math.tan(math.pi / SIDES)                           # the facets' feet at the wall
    nut_reach = hex_circumdiameter(S.nut.af + S.nut_fit) / 2.0
    _, z_bottom = min(flange_bolt_points(), key=lambda p: p[1])                          # the wall's bottom screw, its head
    head_r = R_END.screw.head_dia / 2.0
    for x, y in S.screw_points:
        assert abs(y) + S.head_dia / 2.0 <= corner_y - 1.0
        assert S.lap_x + nut_reach + 1.5 <= x <= R_END.wall_x[0] - nut_reach - 1.5
        assert abs(y) - nut_reach >= head_r + 1.0 or R_END.wall_x[0] - R_END.screw.head_h - (x + S.head_dia / 2.0) >= 1.0
        assert z_bottom - head_r > W.z1                                                    # its head over the web's top face
        for hx, hy in motor_hole_points():
            assert math.hypot(x - hx, y - hy) >= S.head_dia / 2.0 + H.pocket_dia / 2.0 + 2.0
        assert math.hypot(x - DEFAULT.motor_x, y) - nut_reach - H.pilot_dia / 2.0 >= 2.0


def test_the_bracket_and_the_fasteners_are_parts_with_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "j2_wall_bracket")
        assert R.NO_REFERENCE[name] in (f"parts/joints/{name}.py:_envelope()", "lib/forearm/link.py:build_wall_bracket(DEFAULT)")
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces == len(S.screw_points)
    assert parts.load("wall_bracket_screws").MASS_G == pytest.approx(
        PARAMS.STEEL_DENSITY * len(S.screw_points) * shcs_volume(S.screw, S.screw_len))
    assert parts.load("wall_bracket_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * len(S.screw_points) * nut_volume(S.nut))


@pytest.mark.slow
def test_the_forearm_prints_in_two_parts():
    """j2_link and the bracket: one solid each, sharing nothing; together the whole forearm but the lap's holes - the
    wall and the gussets the bracket's, the tongue over the lap plane the bracket's and the web's end under it j2_link's;
    every round feature of each with its axis along its print direction (the bracket's along X: the wall's holes, the
    recess, the cable bore; j2_link's along Z). The lap's holes through both, the counterbores in j2_link's underside,
    the nuts' pockets in the tongue's top face."""
    web, bracket, whole = built.part("j2_link"), built.part("j2_wall_bracket"), _build_whole(DEFAULT)
    assert web.is_valid and bracket.is_valid and len(web.solids()) == len(bracket.solids()) == 1
    assert R.solid_volume(web) == pytest.approx(J2_LINK_VOLUME, abs=0.5)
    assert interference(web, bracket) < 1e-6
    bolt, head = math.pi * (S.bolt_dia / 2.0) ** 2, math.pi * (S.head_dia / 2.0) ** 2
    pocket = math.sqrt(3.0) / 2.0 * (S.nut.af + S.nut_fit) ** 2
    holes = len(S.screw_points) * (bolt * W.thickness + (head - bolt) * S.screw.head_h + (pocket - bolt) * S.nut.h)
    assert R.solid_volume(whole) - R.solid_volume(web) - R.solid_volume(bracket) == pytest.approx(holes, rel=2e-3)
    # which side of the cut: the wall, a gusset, the tongue, the web under it, the web past the lap
    x0, z_lap = R_END.wall_x[0], lap_z()
    gusset_y = sum(R_END.rib_y) / 2.0
    for probe, owner in (((x0 + 4.0, 0.0, 48.0), bracket), ((x0 - 3.0, gusset_y, W.z1 + 5.0), bracket),
                         ((x0 - 3.0, 0.0, z_lap + 1.0), bracket), ((x0 - 3.0, 0.0, z_lap - 1.0), web),
                         ((S.lap_x - 1.0, 0.0, z_lap + 1.0), web), ((S.lap_x + 1.0, 4.0, z_lap + 1.0), bracket)):
        assert is_inside(whole, *probe) and is_inside(owner, *probe), probe
        assert not is_inside(web if owner is bracket else bracket, *probe), probe
    assert bracket.bounding_box().min.X == pytest.approx(x0 - R_END.rib_len, abs=1e-6)
    assert web.bounding_box().max.X == pytest.approx(x0, abs=1e-6) and web.bounding_box().min.Z == pytest.approx(W.z0)
    # the lap's holes, counterbores, pockets
    corner = (S.nut.af + S.nut_fit) / math.sqrt(3.0)
    for x, y in S.screw_points:
        assert not is_inside(web, x, y, z_lap - 1.0) and not is_inside(bracket, x, y, z_lap + 1.0)                 # the hole
        assert is_inside(web, x + S.bolt_dia / 2.0 + 0.3, y, z_lap - 1.0)
        assert not is_inside(web, x + S.head_dia / 2.0 - 0.2, y, W.z0 + 0.5)                                       # the counterbore ...
        assert is_inside(web, x + S.head_dia / 2.0 + 0.2, y, W.z0 + 0.5) and is_inside(web, x + S.head_dia / 2.0 - 0.2, y, lap_screw_head_z() + 0.3)
        assert not is_inside(bracket, x + corner - 0.2, y, W.z1 - 0.5) and is_inside(bracket, x + corner + 0.2, y, W.z1 - 0.5)   # the pocket ...
        assert is_inside(bracket, x + corner - 0.2, y, lap_nut_z() - 0.3)                                          # ... and its floor


@pytest.mark.slow
def test_the_lap_fasteners_fit():
    """In j2_link's frame: the heads in their counterbores, on their floors; the nuts in their pockets, on their floors;
    nothing overlapping the web, the bracket, the wrist motor or its screws (which pass through the lap beside them)."""
    into = "j2_link#1"
    web, bracket = built.part("j2_link"), built.placed("j2_wall_bracket#1", into=into)
    screws, nuts = built.placed("wall_bracket_screws#1", into=into), built.placed("wall_bracket_nuts#1", into=into)
    near = [web, bracket, built.placed("nema17_40mm#3", into=into), built.placed("wrist_motor_screws#1", into=into)]
    for part in (screws, nuts):
        for other in near + [screws, nuts]:
            if other is not part:
                assert interference(part, other) < 0.5
    assert closest_points(screws, web).distance < 0.01 and closest_points(nuts, bracket).distance < 0.01
    bb_s, bb_n = screws.bounding_box(), nuts.bounding_box()
    assert bb_s.min.Z == pytest.approx(W.z0, abs=1e-6) and bb_s.max.Z == pytest.approx(W.z1, abs=1e-6)
    assert bb_n.min.Z == pytest.approx(lap_nut_z(), abs=1e-6) and bb_n.max.Z == pytest.approx(W.z1, abs=1e-6)
