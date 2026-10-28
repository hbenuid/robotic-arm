"""The forearm roll frame's removable back panel (lib/forearm/params.py BackPanelParams): a printed plate
(forearm_roll_back_panel) flush with the frame's rear face in a step across it, over the shaft's bay and the opening
grown down below the bay's bore, held by 4 magnet pairs (roll_back_magnets). Both are designed here with no reference
(lib/reference.py NO_REFERENCE) - THIS file locks their numbers (pieces, volume, bounding box, measured on the build and
written in) and what they are: the panel's outer face on the rear line (the elbow motor's board passes it as it passed
the rear line before the bay was bored through the back), run_gap off the turning shaft and its lug, located by its rim
in the opening; the frame's step, opening and pockets; each magnet pressed to its pocket's bottom, the pair facing
across the step's floor - and all of it clear of the rest of the drive. The elbow motor and its board against the
whole module over the elbow's range: tests/forearm/test_roll_drive.py."""
import math

import pytest
from cadgen.geometry import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.forearm import DEFAULT, motor_pocket, stack_positions
from lib.forearm.roll import back_face, back_magnet_depths, back_point, rear_line
from lib.geom import cylinder
from tests import built
from tests.helpers import interference, is_inside

D, S = DEFAULT.drive, stack_positions(DEFAULT)
B = D.back
PANEL, MAGNETS, FRAME = "forearm_roll_back_panel", "roll_back_magnets", "forearm_roll_block"
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    PANEL: (1, 13702.812, (-29.9, -50.981139, -38.230792), (59.8, 80.881139, 42.878529)),
    MAGNETS: (8, 452.389, (-26.5, -48.528686, -36.167118), (53.0, 75.154777, 40.572188)),
}


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def _out(point) -> float:
    """How far a module point stands out past the rear line (negative: inside it)."""
    (nu, nv), dist = rear_line(DEFAULT)
    return nu * (point.Y - S["y_elbow"]) + nv * point.Z - dist


def test_they_are_designed_here_with_no_reference_on_the_stator():
    assert parts.GROUPS[PANEL] == parts.GROUPS[MAGNETS] == "joints"
    assert not parts.bought(PANEL) and parts.bought(MAGNETS)
    assert R.NO_REFERENCE[PANEL] == "lib/forearm/roll.py:build_back_panel(DEFAULT)"
    assert R.NO_REFERENCE[MAGNETS] == f"parts/joints/{MAGNETS}.py:_envelope()"
    assert parts.load(PANEL).REFERENCE is None and PANEL not in parts.unplaced() and MAGNETS not in parts.unplaced()
    assert PANEL in M.BODIES["stator"] and MAGNETS in M.BODIES["stator"]   # they stay with the frame: elbow_link
    rows = {part: (role, pos) for part, role, pos in M.OCCURRENCES if part in (PANEL, MAGNETS)}
    assert rows == {PANEL: (None, (0.0, 0.0, 0.0)), MAGNETS: (None, (0.0, 0.0, 0.0))}   # built at their stations


def test_the_magnets():
    """A pair at each point, the disc's datasheet size, the mass the modelled NdFeB; each disc in a pocket 0.2 wider and
    0.1 deeper than it, the panel's pocket leaving 0.4 of the panel outside it."""
    mod = parts.load(MAGNETS)
    assert mod.PURCHASE_QTY == 2 * len(B.magnet_points) == LOCKS[MAGNETS][0] == 8
    assert (B.magnet_dia, B.magnet_t) == (6.0, 2.0) and mod.PURCHASE_SPEC == "6 x 2 mm N52 NdFeB disc magnet"
    assert mod.MASS_G == PARAMS.ROLL_BACK_MAGNETS_MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * 8 * math.pi * 3.0**2 * 2.0)
    assert B.pocket_dia - B.magnet_dia == pytest.approx(0.2) and B.pocket_depth - B.magnet_t == pytest.approx(0.1)
    assert B.t - B.pocket_depth == pytest.approx(0.4)
    frame_depth, panel_depth = back_magnet_depths(DEFAULT)
    assert frame_depth + B.magnet_t == pytest.approx(B.t + B.pocket_depth) and panel_depth == pytest.approx(B.t - B.pocket_depth)
    assert frame_depth - (panel_depth + B.magnet_t) == pytest.approx(0.2)   # the pair's faces across the step's floor


def test_the_rear_face_frame():
    """back_face: the roll axis' point on the rear line, n its normal, t along it; back_point walks the face."""
    z0, (nu, nv), (tu, tv) = back_face(DEFAULT)
    (lu, lv), dist = rear_line(DEFAULT)
    assert (nu, nv) == (lu, lv) and math.isclose(nu * tu + nv * tv, 0.0, abs_tol=1e-12) and tu > 0.0
    for x, s in ((0.0, 0.0), (20.0, -40.0), (-10.0, 25.0)):
        x0, y0, zz = back_point(x, s, 0.0, DEFAULT)
        assert x0 == x and math.isclose(nu * (y0 - S["y_elbow"]) + nv * zz, dist, abs_tol=1e-9)
        assert math.isclose(nu * (back_point(x, s, 2.0, DEFAULT)[1] - S["y_elbow"]) + nv * back_point(x, s, 2.0, DEFAULT)[2], dist - 2.0, abs_tol=1e-9)
    assert back_point(0.0, 0.0, 0.0, DEFAULT)[1] == 0.0 and math.isclose(back_point(0.0, 0.0, 0.0, DEFAULT)[2], z0)


@pytest.mark.slow
@pytest.mark.parametrize("name", [PANEL, MAGNETS])
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    if name == MAGNETS:
        assert R.solid_volume(shape) == pytest.approx(solids * math.pi * (B.magnet_dia / 2.0) ** 2 * B.magnet_t, rel=1e-6)


@pytest.mark.slow
def test_the_frames_step_opening_and_pockets():
    """The step across the rear face (t deep, its bottom edge at `bottom`; the face whole below it, at the rear line's
    probe too); the opening grown down below the bay's bore, opening_depth in, the frame solid past it and below its
    bottom, 8 or more of the frame left over the motor's pocket; the magnets' pockets in the step's floor, 3 or more
    of wall to the bay's bore and the side faces."""
    block = built.part(FRAME)
    for x, s in ((0.0, -50.0), (25.0, 10.0), (-25.0, 30.0)):
        assert not is_inside(block, *back_point(x, s, B.t - 0.2, DEFAULT)) and is_inside(block, *back_point(x, s, B.t + 0.2, DEFAULT)), (x, s)
    assert is_inside(block, *back_point(0.0, B.bottom - 1.0, 0.5, DEFAULT)) and is_inside(block, *back_point(0.0, -60.0, 0.5, DEFAULT))
    for x in (-24.0, 0.0, 24.0):
        assert not is_inside(block, *back_point(x, -38.0, B.opening_depth - 0.5, DEFAULT))
        assert is_inside(block, *back_point(x, -38.0, B.opening_depth + 0.5, DEFAULT))
    assert is_inside(block, *back_point(0.0, B.opening_bottom - 0.5, B.opening_depth - 0.5, DEFAULT))
    assert is_inside(block, *back_point(D.bay_r + 0.5, -20.0, B.t + 1.0, DEFAULT))   # the side wall beside it
    # the opening's floor at its bottom edge (its point nearest the motor's pocket, in the pocket's z range) over the
    # pocket's top
    _y0, y1, z0, z1 = motor_pocket(DEFAULT)
    _x, y, z = back_point(0.0, B.opening_bottom, B.opening_depth, DEFAULT)
    assert z0 < z < z1 and y - y1 >= 8.0
    for x, s in B.magnet_points:
        assert not is_inside(block, *back_point(x, s, B.t + B.pocket_depth - 0.1, DEFAULT)) and is_inside(block, *back_point(x, s, B.t + B.pocket_depth + 0.1, DEFAULT))
        assert D.block_x[1] - (abs(x) + B.pocket_dia / 2.0) >= 3.0
        for depth in (B.t, B.t + B.pocket_depth):
            for k in range(36):
                a = 2.0 * math.pi * k / 36
                _x, y, z = back_point(x + B.pocket_dia / 2.0 * math.cos(a), s + B.pocket_dia / 2.0 * math.sin(a), depth, DEFAULT)
                if z < S["z_bearing_1"]:
                    assert math.hypot(_x, y) - D.bay_r >= 3.0, (x, s)


@pytest.mark.slow
def test_the_panel_is_flush_on_the_rear_line_and_clear_of_the_shaft(module):
    """Its outer face on the rear line, nothing of it past it; on the step's floor (contact, no overlap with the frame);
    run_gap or more off the shaft's rear face and its lug's sweep at any roll (a disc stop_lug_r[1] round the roll axis
    from the shaft's rear face), its thinnest over the bay's bore's top edge; its rim in the opening, rim_clear off its
    walls."""
    panel, frame, shaft = _leaf(module, PANEL), _leaf(module, FRAME), _leaf(module, "forearm_roll_shaft")
    outs = [_out(v) for v in panel.vertices()]
    assert max(outs) == pytest.approx(0.0, abs=1e-6)
    assert interference(panel, frame) == pytest.approx(0.0, abs=1e-6) and closest_points(panel, frame).distance < 1e-6
    sweep = cylinder(D.stop_lug_r[1], D.stop_lug_t, z0=S["z_shaft_end"])
    assert closest_points(panel, sweep).distance == pytest.approx(D.run_gap, abs=1e-6)
    assert closest_points(panel, shaft).distance >= D.run_gap
    # the rim: rim_w wide, rim_clear inside the opening's side walls (x = +/-bay_r) and its bottom, rim_h tall
    mid, h = D.bay_r - B.rim_clear - B.rim_w / 2.0, B.t + B.rim_h / 2.0
    for x in (-mid, mid):
        assert is_inside(panel, *back_point(x, -30.0, h, DEFAULT)) and not is_inside(panel, *back_point(x, B.rim_top + 0.5, h, DEFAULT))
        assert not is_inside(panel, *back_point(math.copysign(D.bay_r - B.rim_clear + 0.05, x), -30.0, h, DEFAULT))
        assert not is_inside(panel, *back_point(math.copysign(mid - B.rim_w / 2.0 - 0.05, x), -30.0, h, DEFAULT))
    assert is_inside(panel, *back_point(0.0, B.opening_bottom + B.rim_clear + B.rim_w / 2.0, h, DEFAULT))
    assert not is_inside(panel, *back_point(0.0, -30.0, B.t + B.rim_h + 0.05, DEFAULT))
    assert panel.bounding_box().min.X == pytest.approx(-(D.block_x[1] - B.side_clear)) and panel.bounding_box().max.Y == pytest.approx(D.tower_y - B.side_clear)


@pytest.mark.slow
def test_the_magnets_sit_in_their_pockets_clear_of_the_drive(module):
    """Each disc on its pocket's bottom (touching its own part: the frame's in the step's floor, the panel's in its
    inner face), the pair 0.2 apart across the step's floor; no interference with anything in the drive."""
    magnets, panel, frame = _leaf(module, MAGNETS), _leaf(module, PANEL), _leaf(module, FRAME)
    frame_depth, panel_depth = back_magnet_depths(DEFAULT)
    for solid in magnets.solids():
        c = solid.center()
        depth = -_out(c) - B.magnet_t / 2.0          # its outer face's depth in from the rear line
        own = frame if depth > B.t else panel
        assert depth == pytest.approx(frame_depth if own is frame else panel_depth, abs=1e-6)
        assert closest_points(solid, own).distance < 1e-6
    for other in module.children:
        if other is not magnets:
            assert interference(magnets, other) == pytest.approx(0.0, abs=1e-6), other.label


@pytest.mark.slow
def test_the_panel_clears_the_rest_of_the_drive(module):
    panel = _leaf(module, PANEL)
    for other in module.children:
        if other is not panel:
            assert interference(panel, other) == pytest.approx(0.0, abs=1e-6), other.label
