"""The forearm roll rotor's clamp (lib/forearm/params.py RollDriveParams clamp_*): 4x M4 SHCS (roll_clamp_screws) from
counterbores in the forearm wall's wrist face (j2_wall_bracket) through the wall, the pulley, both 6806 inner rings and
the shaft into 4 M4 nuts (roll_clamp_nuts) in the shaft's flange - the wall, the pulley, the inner rings and the shaft
one body; the screws centre the wall on the roll axis. Purchased parts with no reference (lib/reference.py
NO_REFERENCE), which keep no reference file - THIS file locks their numbers (the pieces, the modelled volume and the
bounding box, measured on the build and written in) and what they are: the screw clamp_screw_len under the head, on the
clamp's circle; the nut a corner radial. The holes through the pulley and the shaft and the nut pockets are the printed
parts' (tests/forearm/test_roll_drive.py probes them); the wall's counterbores are here. In the module: on the rotor;
the heads on the wall's counterbores' floors, clamp_head_clear under its wrist face, the tips at the shaft's rear face,
the nuts on their pockets' ends 2 pitches short of the tips, held from turning by the pockets' flats - and clear of
everything else in the drive and of the forearm."""
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
from lib.forearm import DEFAULT, clamp_points, flange_bolt_points, stack_positions
from lib.forearm.layout import module_frame_in_host
from tests import built
from tests.forearm.helpers import j2_link_in_drive_frame
from tests.helpers import interference, is_inside

D, W, S = DEFAULT.drive, DEFAULT.roll_end, stack_positions(DEFAULT)
SCREWS, NUTS = "roll_clamp_screws", "roll_clamp_nuts"
HEAD_R = D.clamp_screw.head_dia / 2.0 + D.clamp_head_clear      # the wall's counterbores
SEAT = D.clamp_screw.head_h + D.clamp_head_clear                # ... their depth from its wrist face
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    SCREWS: (4, 3066.672, (-11.631728, -11.631728, -4.0), (23.263456, 23.263456, 54.0)),
    NUTS: (4, 382.322, (-12.035471, -12.035471, 0.0), (24.070941, 24.070941, 3.2)),
}


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def test_they_are_purchased_parts_with_no_reference_on_the_rotor():
    for name in (SCREWS, NUTS):
        mod = parts.load(name)
        assert parts.GROUPS[name] == "joints" and parts.bought(name) and mod.PURCHASE_QTY == len(clamp_points(DEFAULT)) == LOCKS[name][0]
        assert R.NO_REFERENCE[name] == f"parts/joints/{name}.py:_envelope()" and name not in parts.unplaced()
        assert name in M.BODIES["rotor"], name   # they turn with the pulley and the shaft: forearm_link
    # one row each in the drive: the screws turned over (the heads up the roll axis) on the counterbores' floor, the
    # nuts at their far faces' station
    rows = {part: (role, pos) for part, role, pos in M.OCCURRENCES if part in (SCREWS, NUTS)}
    assert rows == {SCREWS: (None, ((0.0, 0.0, S["z_clamp_head"]), (180.0, 0.0, 0.0))), NUTS: (None, (0.0, 0.0, S["z_clamp_nut"]))}
    # the forearm wall has no screws of its own: these are its
    assert not any(part.startswith("roll_wall") for part, _, _ in M.OCCURRENCES)


def test_the_clamps_screw_and_nut():
    """ISO 4762 M4, clamp_screw_len under the head (a stock length); ISO 4032 M4; both on the clamp's circle (the
    screws' pattern mirrored in y: the row turns it over about X); the masses the modelled steel."""
    screws, nuts = parts.load(SCREWS), parts.load(NUTS)
    assert screws.SCREW == D.clamp_screw == PARAMS.M4_SHCS and nuts.NUT == D.clamp_nut == PARAMS.M4_NUT
    assert screws.LENGTH == D.clamp_screw_len == 50.0
    assert screws.PURCHASE_SPEC == "M4 x 50 socket head cap screw (ISO 4762)" and nuts.PURCHASE_SPEC == "M4 hex nut (ISO 4032)"
    assert sorted((x, -y) for x, y in screws.POINTS) == sorted(clamp_points(DEFAULT)) == sorted(nuts.POINTS)
    assert all(math.isclose(math.hypot(x, y), D.clamp_circle_dia / 2.0) for x, y in nuts.POINTS)
    n = LOCKS[SCREWS][0]
    assert screws.MASS_G == PARAMS.ROLL_CLAMP_SCREWS_MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * n * shcs_volume(D.clamp_screw, 50.0))
    assert nuts.MASS_G == PARAMS.ROLL_CLAMP_NUTS_MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * n * nut_volume(D.clamp_nut))


def test_the_stations():
    """The heads' bearing faces on the wall's counterbores' floor, their tops clamp_head_clear under its wrist face; the
    tips at the shaft's rear face; each nut 2 pitches short of its tip, its bearing face on its pocket's end."""
    head_top = S["z_clamp_head"] + D.clamp_screw.head_h
    assert math.isclose(S["z_wall_back"] - head_top, D.clamp_head_clear) and S["z_wall_back"] == -W.wall_x[0]
    assert math.isclose(S["z_clamp_head"] - D.clamp_screw_len, S["z_shaft_end"])
    assert math.isclose(S["z_clamp_nut"] - S["z_shaft_end"], 2 * PARAMS.M4_PITCH)
    assert math.isclose(S["z_clamp_nut"] + D.clamp_nut.h, S["z_clamp_nut_seat"])
    # the wall's holes are the clamp's, seen from j2_link (host y = module y, host z = module x + axis_z)
    assert sorted(flange_bolt_points(DEFAULT)) == sorted((y, x + W.axis_z) for x, y in clamp_points(DEFAULT))
    # the wall under the heads, and round the counterbores to its cable hole (the rotor's bore)
    assert (W.wall_x[1] - W.wall_x[0]) - SEAT >= 3.5
    assert D.clamp_circle_dia / 2.0 - HEAD_R - D.bore / 2.0 >= 1.5 - 1e-9


@pytest.mark.slow
@pytest.mark.parametrize("name", [SCREWS, NUTS])
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    one = shcs_volume(D.clamp_screw, D.clamp_screw_len) if name == SCREWS else nut_volume(D.clamp_nut)
    assert R.solid_volume(shape) == pytest.approx(solids * one, rel=1e-6)


def _probe(hole, d: float, x: float) -> tuple[float, float, float]:
    """A point in j2_link's frame at x, `d` out from a wall hole's axis (y, z) away from the roll axis."""
    y, z = hole
    r = math.hypot(y, z - W.axis_z)
    return x, y + d * y / r, z + d * (z - W.axis_z) / r


@pytest.mark.slow
def test_the_walls_counterbores():
    """j2_wall_bracket: at each hole a counterbore HEAD_R round, SEAT deep from the wrist face, the M4 clearance hole on
    through the wall; the wall whole round it."""
    bracket = built.part("j2_wall_bracket")
    x0, x_floor = W.wall_x[0], W.wall_x[0] + SEAT
    for hole in flange_bolt_points(DEFAULT):
        assert not is_inside(bracket, *_probe(hole, HEAD_R - 0.2, x_floor - 0.2))     # in the counterbore
        assert is_inside(bracket, *_probe(hole, HEAD_R - 0.2, x_floor + 0.2))         # its floor
        assert is_inside(bracket, *_probe(hole, HEAD_R + 0.2, x0 + 0.2))              # the wall round it
        assert not is_inside(bracket, *_probe(hole, 0.0, x_floor + 1.0))              # the hole on through
        assert not is_inside(bracket, *_probe(hole, D.clamp_bolt_dia / 2.0 - 0.1, x_floor + 1.0))
        assert is_inside(bracket, *_probe(hole, D.clamp_bolt_dia / 2.0 + 0.1, x_floor + 1.0))


@pytest.mark.slow
def test_in_place_the_clamp_sits_on_its_seats_clear_of_the_drive(module):
    """The placed patterns: the heads from the counterbores' floors to clamp_head_clear under the wall's wrist face, the
    tips at the shaft's rear face, the nuts between their stations, bearing on the shaft; the shanks clear of the
    pulley's and the shaft's holes; no interference with anything else in the drive."""
    screws, nuts = _leaf(module, SCREWS), _leaf(module, NUTS)
    pulley, shaft = _leaf(module, "forearm_roll_pulley"), _leaf(module, "forearm_roll_shaft")
    bb = screws.bounding_box()
    assert bb.max.Z == pytest.approx(S["z_wall_back"] - D.clamp_head_clear, abs=1e-6) and bb.min.Z == pytest.approx(S["z_shaft_end"], abs=1e-6)
    bb = nuts.bounding_box()
    assert bb.min.Z == pytest.approx(S["z_clamp_nut"], abs=1e-6) and bb.max.Z == pytest.approx(S["z_clamp_nut_seat"], abs=1e-6)
    assert closest_points(nuts, shaft).distance < 0.01
    hole_gap = (D.clamp_bolt_dia - D.clamp_screw.d) / 2.0
    assert closest_points(screws, shaft).distance == pytest.approx(hole_gap, abs=0.01)
    assert closest_points(screws, pulley).distance == pytest.approx(hole_gap, abs=0.01)
    for part in (screws, nuts):
        for other in module.children:
            if other is not part:
                assert interference(part, other) == pytest.approx(0.0, abs=1e-6), (part.label, other.label)


@pytest.mark.slow
def test_on_the_forearm_the_heads_sit_in_the_wall():
    """In the drive's mount frame, against the forearm as the arm turns it (j2_link with its wall bracket): the heads on
    the counterbores' floors, their tops clamp_head_clear under the wall's wrist face; no overlap with the forearm."""
    screws = built.leaf(M.forearm_roll_drive, SCREWS).moved(to_location(module_frame_in_host(DEFAULT)))
    forearm = j2_link_in_drive_frame()
    assert screws.bounding_box().min.X == pytest.approx(W.wall_x[0] + D.clamp_head_clear, abs=1e-6)   # module +Z = host -X
    assert closest_points(screws, forearm).distance < 0.01
    assert interference(screws, forearm) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.slow
def test_the_pockets_hold_the_nuts_from_turning(module):
    """Each nut a corner radial in its pocket's slot, af wide: a few degrees about its own axis runs its corners into
    the shaft's flange."""
    nuts, shaft = _leaf(module, NUTS), _leaf(module, "forearm_roll_shaft")
    for solid, (x, y) in zip(sorted(nuts.solids(), key=lambda s: math.atan2(s.center().Y, s.center().X)),
                             sorted(clamp_points(DEFAULT), key=lambda p: math.atan2(p[1], p[0])), strict=True):
        assert math.isclose(solid.center().X, x, abs_tol=1e-6) and math.isclose(solid.center().Y, y, abs_tol=1e-6)
        turned = solid.rotate(Axis((x, y, 0.0), (0.0, 0.0, 1.0)), 5.0)
        assert interference(turned, shaft) > 0.1, (x, y)
