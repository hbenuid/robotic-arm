"""The forearm wall's fasteners (lib/forearm/params.py RollEndParams screw / head_* / screw_len): 4x M3 SHCS
(roll_wall_screws) from counterbores in the wall's wrist face (j2_wall_bracket) through the wall and the roll pulley's
spigot into 4 M3 nuts (roll_wall_nuts) in the pulley's core - each clamps the wall onto the ring's face. Purchased parts
with no reference (lib/reference.py NO_REFERENCE), which keep no reference file - THIS file locks their numbers (the
pieces, the modelled volume and the bounding box, measured on the build and written in) and what they are: the screw
screw_len under the head, on the wall's bolt circle; the nut a corner radial. The nut pockets and the pulley's holes
are tests/forearm/test_roll_drive.py's; the wall's counterbores are here. In the module: on the rotor; the heads'
bearing faces head_seat under the wall's wrist face (flush), the tips flush with the nuts' far faces, the nuts on their
pockets' ends, held from turning by the pockets' flats - clear of everything else in the drive and of the forearm."""
import math

import pytest
from build123d import Axis
from cad.build123d import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.datum import to_location
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import DEFAULT, flange_bolt_points, flange_bolt_points_module, stack_positions
from lib.forearm.layout import module_frame_in_host
from tests import built
from tests.forearm.helpers import j2_link_in_drive_frame
from tests.helpers import interference, is_inside

W, D, S = DEFAULT.roll_end, DEFAULT.drive, stack_positions(DEFAULT)
SCREWS, NUTS = "roll_wall_screws", "roll_wall_nuts"
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    SCREWS: (4, 596.246, (-18.25, -18.25, -3.0), (36.5, 36.5, 15.0)),
    NUTS: (4, 183.635, (-18.675426, -18.675426, 0.0), (37.350853, 37.350853, 2.4)),
}


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def test_they_are_purchased_parts_with_no_reference_on_the_rotor():
    for name in (SCREWS, NUTS):
        mod = parts.load(name)
        assert parts.GROUPS[name] == "joints" and parts.bought(name) and mod.PURCHASE_QTY == W.bolt_count == LOCKS[name][0]
        assert R.NO_REFERENCE[name] == f"parts/joints/{name}.py:_envelope()" and name not in parts.unplaced()
        assert name in M.BODIES["rotor"], name   # they turn with the pulley and the forearm: forearm_link
    # one row each in the drive: the screws turned over (the heads up the roll axis) on the counterbores' floor, the
    # nuts at their far faces' station
    rows = {part: (role, pos) for part, role, pos in M.OCCURRENCES if part in (SCREWS, NUTS)}
    assert rows == {SCREWS: (None, ((0.0, 0.0, S["z_end_head"]), (180.0, 0.0, 0.0))), NUTS: (None, (0.0, 0.0, S["z_end_nut"]))}


def test_the_walls_screw_and_nut():
    """ISO 4762 M3 x 12; ISO 4032 M3; both on the wall's bolt circle (the screws' pattern mirrored in y: the row turns
    it over about X); the masses the modelled steel."""
    screws, nuts = parts.load(SCREWS), parts.load(NUTS)
    assert screws.SCREW == W.screw == PARAMS.M3_SHCS and nuts.NUT == D.end_nut == PARAMS.M3_NUT
    assert screws.LENGTH == W.screw_len == 12.0
    assert screws.PURCHASE_SPEC == "M3 x 12 socket head cap screw (ISO 4762)" and nuts.PURCHASE_SPEC == "M3 hex nut (ISO 4032)"
    assert sorted((x, -y) for x, y in screws.POINTS) == sorted(flange_bolt_points_module(DEFAULT)) == sorted(nuts.POINTS)
    assert all(math.isclose(math.hypot(x, y), W.bolt_circle_dia / 2.0) for x, y in nuts.POINTS)
    n = LOCKS[SCREWS][0]
    assert screws.MASS_G == PARAMS.ROLL_WALL_SCREWS_MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * n * shcs_volume(W.screw, 12.0))
    assert nuts.MASS_G == PARAMS.ROLL_WALL_NUTS_MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * n * nut_volume(D.end_nut))


def test_the_stations():
    """The heads flush with the wall's wrist face in counterbores head_seat deep (the head's height), wide enough for
    it, 2.7 of the wall left under them at the holes (the recess on the elbow face over them); the tips flush with the
    nuts' far faces, the whole nut on the thread; the nut's bearing face on its pocket's end."""
    assert W.head_seat == W.screw.head_h == 3.0 and W.head_dia == 5.7 and W.head_dia > W.screw.head_dia
    assert math.isclose(S["z_wall_back"] - S["z_end_head"], W.head_seat)
    wall_at_holes = (W.wall_x[1] - W.wall_x[0]) - (W.flange_recess_depth + W.flange_recess_clear)
    assert all(math.hypot(y, z - W.axis_z) + W.bolt_dia / 2.0 < (W.flange_dia + W.flange_recess_add) / 2.0
               for y, z in flange_bolt_points(DEFAULT))                   # every hole inside the recess
    assert math.isclose(wall_at_holes - W.head_seat, 2.7)
    assert math.isclose(S["z_end_head"] - W.screw_len, S["z_end_tip"]) and S["z_end_tip"] == S["z_end_nut"]
    assert math.isclose(S["z_end_nut"] + D.end_nut.h, S["z_end_nut_seat"])


@pytest.mark.slow
def test_the_walls_counterbores():
    """j2_wall_bracket: at each hole a counterbore head_dia wide, head_seat deep from the wrist face, the M3 clearance
    hole on through the wall; the wall whole round it."""
    bracket = built.part("j2_wall_bracket")
    x0, x_floor = W.wall_x[0], W.wall_x[0] + W.head_seat
    for hole in flange_bolt_points(DEFAULT):
        assert not is_inside(bracket, *_probe(hole, W.head_dia / 2.0 - 0.2, x_floor - 0.2))     # in the counterbore
        assert is_inside(bracket, *_probe(hole, W.head_dia / 2.0 - 0.2, x_floor + 0.2))         # its floor
        assert is_inside(bracket, *_probe(hole, W.head_dia / 2.0 + 0.2, x0 + 0.2))              # the wall round it
        assert not is_inside(bracket, *_probe(hole, 0.0, x_floor + 1.0))                        # the hole on through
        assert not is_inside(bracket, *_probe(hole, W.bolt_dia / 2.0 - 0.1, x_floor + 1.0))
        assert is_inside(bracket, *_probe(hole, W.bolt_dia / 2.0 + 0.1, x_floor + 1.0))


def _probe(hole, d: float, x: float) -> tuple[float, float, float]:
    """A point in j2_link's frame at x, `d` out from a wall hole's axis (y, z) away from the roll axis."""
    y, z = hole
    r = math.hypot(y, z - W.axis_z)
    return x, y + d * y / r, z + d * (z - W.axis_z) / r


@pytest.mark.slow
@pytest.mark.parametrize("name", [SCREWS, NUTS])
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    one = shcs_volume(W.screw, W.screw_len) if name == SCREWS else nut_volume(D.end_nut)
    assert R.solid_volume(shape) == pytest.approx(solids * one, rel=1e-6)


@pytest.mark.slow
def test_in_the_drive_they_sit_on_their_seats_clear_of_it(module):
    """The placed patterns: the heads' tops at the wall's wrist face's station, the tips at the nuts' far faces; the
    nuts between their stations, bearing on the pulley's pockets; the shanks clear of the pulley's holes; no
    interference with anything else in the drive."""
    screws, nuts, pulley = _leaf(module, SCREWS), _leaf(module, NUTS), _leaf(module, "forearm_roll_pulley")
    bb = screws.bounding_box()
    assert bb.max.Z == pytest.approx(S["z_wall_back"], abs=1e-6) and bb.min.Z == pytest.approx(S["z_end_tip"], abs=1e-6)
    bb = nuts.bounding_box()
    assert bb.min.Z == pytest.approx(S["z_end_nut"], abs=1e-6) and bb.max.Z == pytest.approx(S["z_end_nut_seat"], abs=1e-6)
    assert closest_points(nuts, pulley).distance < 0.01
    assert closest_points(screws, pulley).distance == pytest.approx((W.bolt_dia - W.screw.d) / 2.0, abs=0.01)
    for part in (screws, nuts):
        for other in module.children:
            if other is not part:
                assert interference(part, other) == pytest.approx(0.0, abs=1e-6), (part.label, other.label)


@pytest.mark.slow
def test_on_the_forearm_the_heads_sit_flush_in_the_wall():
    """In the drive's mount frame, against the forearm as the arm turns it (j2_link with its wall bracket): the heads on
    the counterbores' floors, their tops flush with the wall's wrist face; no overlap with the forearm."""
    screws = built.leaf(M.forearm_roll_drive, SCREWS).moved(to_location(module_frame_in_host(DEFAULT)))
    forearm = j2_link_in_drive_frame()
    assert screws.bounding_box().min.X == pytest.approx(W.wall_x[0], abs=1e-6)       # module +Z = host -X
    assert closest_points(screws, forearm).distance < 0.01
    assert interference(screws, forearm) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.slow
def test_the_pockets_hold_the_nuts_from_turning(module):
    """Each nut a corner radial in its pocket's slot, af wide: a few degrees about its own axis runs its corners into
    the pulley's core."""
    nuts, pulley = _leaf(module, NUTS), _leaf(module, "forearm_roll_pulley")
    for solid, (x, y) in zip(sorted(nuts.solids(), key=lambda s: math.atan2(s.center().Y, s.center().X)),
                             sorted(flange_bolt_points_module(DEFAULT), key=lambda p: math.atan2(p[1], p[0])), strict=True):
        assert math.isclose(solid.center().X, x, abs_tol=1e-6) and math.isclose(solid.center().Y, y, abs_tol=1e-6)
        turned = solid.rotate(Axis((x, y, 0.0), (0.0, 0.0, 1.0)), 5.0)
        assert interference(turned, pulley) > 0.1, (x, y)
