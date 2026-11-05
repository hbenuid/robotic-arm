"""The wrist frame's removable back panel (lib/wrist/params.py WristBackPanelParams): a printed plate (wrist_back_panel)
flush with wrist_link's slanted rear in a step over the tower, over the shaft's bay bored through the rear, held by 4
magnet pairs (wrist_back_magnets) - the forearm roll frame's (tests/forearm/test_roll_back_panel.py) one joint further
out. Both are designed here with no reference (lib/reference.py NO_REFERENCE) - THIS file locks their numbers (pieces,
volume, bounding box, measured on the build and written in) and what they are: the panel's outer face on the rear plane,
one plane (it prints flat), on the step's floor, run_gap off the turning shaft and its lug, located by its rim in the
bay's bore, its notches open (the gripper servo's cable's, the MKS board's near side plug's room); the frame's step and
pockets; each magnet pressed to its pocket's bottom, the pair facing across the step's floor - and all of it clear of
the rest of the drive."""
import math

import pytest
from build123d import Box, GeomType, Pos
from cadgen.geometry import closest_points

import parts
from assemblies import wrist_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.datum import to_location
from lib.geom import cylinder
from lib.wrist import DEFAULT
from lib.wrist.layout import (
    board_plug_rooms,
    board_window,
    module_frame_in_host,
    rear_line,
    rear_plane,
    stack_positions,
    toward_motor,
)
from lib.wrist.roll import back_magnet_depths, back_point
from tests import built
from tests.helpers import interference, is_inside

D, S = DEFAULT.roll, stack_positions(DEFAULT)
B = D.back
PANEL, MAGNETS = "wrist_back_panel", "wrist_back_magnets"
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    PANEL: (1, 7034.551, (-27.0, -30.0, -18.424332), (57.0, 53.932894, 8.799233)),
    MAGNETS: (8, 113.097, (-23.5, -19.46795, -17.781645), (47.0, 40.289558, 7.025311)),
}


@pytest.fixture(scope="module")
def module():
    return built.model(M.wrist_roll_drive)


@pytest.fixture(scope="module")
def frame():
    """wrist_link (the drive's frame) in the module frame."""
    return built.part("wrist_link").moved(to_location(module_frame_in_host(DEFAULT)).inverse())


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def _out(point) -> float:
    """How far a module point stands out past the rear plane (negative: inside it)."""
    (px, py, pz), (nx, ny, nz) = rear_plane(DEFAULT)
    return nx * (point.X - px) + ny * (point.Y - py) + nz * (point.Z - pz)


def test_they_are_designed_here_with_no_reference_on_the_stator():
    assert parts.GROUPS[PANEL] == parts.GROUPS[MAGNETS] == "wrist"
    assert not parts.bought(PANEL) and parts.bought(MAGNETS) and parts.load(PANEL).ACCENT
    assert R.NO_REFERENCE[PANEL] == "lib/wrist/roll.py:build_back_panel(DEFAULT)"
    assert R.NO_REFERENCE[MAGNETS] == f"parts/wrist/{MAGNETS}.py:_envelope()"
    assert parts.load(PANEL).REFERENCE is None and PANEL not in parts.unplaced() and MAGNETS not in parts.unplaced()
    assert PANEL in M.BODIES["stator"] and MAGNETS in M.BODIES["stator"]   # they stay with the frame: wrist_pitch_link
    rows = {part: (role, pos) for part, role, pos in M.OCCURRENCES if part in (PANEL, MAGNETS)}
    assert rows == {PANEL: (None, (0.0, 0.0, 0.0)), MAGNETS: (None, (0.0, 0.0, 0.0))}   # built at their stations


def test_the_magnets():
    """A pair at each point, the disc's datasheet size, the mass the modelled NdFeB; each disc in a pocket 0.2 wider and
    0.1 deeper than it, the panel's pocket leaving 0.4 of the panel outside it."""
    mod = parts.load(MAGNETS)
    assert mod.PURCHASE_QTY == 2 * len(B.magnet_points) == LOCKS[MAGNETS][0] == 8
    assert (B.magnet_dia, B.magnet_t) == (3.0, 2.0) and mod.PURCHASE_SPEC == "3 x 2 mm N52 NdFeB disc magnet"
    assert mod.MASS_G == PARAMS.WRIST_BACK_MAGNETS_MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * 8 * math.pi * 1.5**2 * 2.0)
    assert B.pocket_dia - B.magnet_dia == pytest.approx(0.2) and B.pocket_depth - B.magnet_t == pytest.approx(0.1)
    assert B.t - B.pocket_depth == pytest.approx(0.4)
    frame_depth, panel_depth = back_magnet_depths(DEFAULT)
    assert frame_depth + B.magnet_t == pytest.approx(B.t + B.pocket_depth) and panel_depth == pytest.approx(B.t - B.pocket_depth)
    assert frame_depth - (panel_depth + B.magnet_t) == pytest.approx(0.2)   # the pair's faces across the step's floor


def test_the_rear_plane():
    """rear_plane: the rear line's plane (it holds the module's X), its normal outward (behind the frame); back_point
    walks it and steps in along the normal."""
    (ya, za), (yb, zb) = rear_line(DEFAULT)
    (px, py, pz), (nx, ny, nz) = rear_plane(DEFAULT)
    assert (px, py, pz) == (0.0, ya, za) and nx == 0.0 and nz < 0.0 and math.isclose(math.hypot(ny, nz), 1.0)
    assert math.isclose(ny * (yb - ya) + nz * (zb - za), 0.0, abs_tol=1e-9)
    for x, y in ((0.0, 0.0), (20.0, -25.0), (-10.0, 15.0)):
        x0, y0, z0 = back_point(x, y, 0.0, DEFAULT)
        assert x0 == x and y0 == y and math.isclose(ny * (y0 - ya) + nz * (z0 - za), 0.0, abs_tol=1e-9)
        _x1, y1, z1 = back_point(x, y, 2.0, DEFAULT)
        assert math.isclose(ny * (y1 - ya) + nz * (z1 - za), -2.0, abs_tol=1e-9)


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
def test_the_frames_step_and_pockets(frame):
    """The step over the tower: t deep, the frame solid past it (at the tower's far side and beside the bay, on both
    sides of R'); not where the plate runs on behind the frame (the step stops at its free face); the bay bored on through
    it; the magnets' pockets in the step's floor, 3 or more of wall to the bay's bore and to the plate's coupler face."""
    for x, y in ((0.0, -27.0), (26.0, 0.0), (-24.5, 0.0), (15.0, -20.0)):
        assert not is_inside(frame, *back_point(x, y, B.t - 0.2, DEFAULT)) and is_inside(frame, *back_point(x, y, B.t + 0.2, DEFAULT)), (x, y)
    assert not is_inside(frame, *back_point(0.0, 0.0, B.t + 0.5, DEFAULT))                     # the bay, on through the step
    s = toward_motor(DEFAULT)
    assert is_inside(frame, *back_point(S["x_free"] - 1.0, s * 15.0, 1.0, DEFAULT))           # the plate: no step in it
    for x, y in B.magnet_points:
        assert not is_inside(frame, *back_point(x, y, B.t + B.pocket_depth - 0.1, DEFAULT)) and is_inside(frame, *back_point(x, y, B.t + B.pocket_depth + 0.1, DEFAULT))
        assert x - B.pocket_dia / 2.0 - S["x_coupler"] >= 3.0
        for depth in (B.t, B.t + B.pocket_depth):
            for k in range(36):
                a = 2.0 * math.pi * k / 36
                px, py, pz = back_point(x + B.pocket_dia / 2.0 * math.cos(a), y + B.pocket_dia / 2.0 * math.sin(a), depth, DEFAULT)
                if pz < S["z_bearing_1"]:
                    assert math.hypot(px, py) - D.bay_r >= 3.0, (x, y)


@pytest.mark.slow
def test_the_panel_is_flush_flat_and_clear_of_the_shaft(module, frame):
    """Its outer face on the rear plane, nothing of it past it, one plane (it prints flat, outer face down); on the step's
    floor (contact, no overlap with the frame); run_gap or more off the shaft and its lug's sweep at any roll; its rim in
    the bay's bore, rim_clear off it; its edge gap short of the board's window, side_clear off the plate."""
    panel, shaft = _leaf(module, PANEL), _leaf(module, "wrist_roll_shaft")
    outs = [_out(v) for v in panel.vertices()]
    assert max(outs) == pytest.approx(0.0, abs=1e-6)
    (_p, (_nx, ny, nz)) = rear_plane(DEFAULT)
    flats = [f for f in panel.faces() if f.geom_type == GeomType.PLANE and abs(f.normal_at().Y - ny) < 1e-6 and abs(f.normal_at().Z - nz) < 1e-6]
    assert len(flats) == 1 and max(abs(_out(v)) for v in flats[0].vertices()) < 1e-6                 # one outer face, on the plane
    assert interference(panel, frame) == pytest.approx(0.0, abs=1e-6) and closest_points(panel, frame).distance < 1e-6
    sweep = cylinder(D.stop_lug_r[1], D.stop_lug_t, z0=S["z_shaft_end"])
    assert closest_points(panel, sweep).distance >= D.run_gap and closest_points(panel, shaft).distance >= D.run_gap
    # the rim: a ring rim_clear inside the bay's bore, rim_w wide, rim_h in from the panel's inner face
    mid = D.bay_r - B.rim_clear - B.rim_w / 2.0
    for x, y in ((-mid, 0.0), (0.0, -mid), (mid, 0.0)):
        assert is_inside(panel, *back_point(x, y, B.t + B.rim_h / 2.0, DEFAULT)), (x, y)
        assert not is_inside(panel, *back_point(x, y, B.t + B.rim_h + 0.2, DEFAULT)), (x, y)
    assert not is_inside(panel, *back_point(D.bay_r - B.rim_clear + 0.05, 0.0, B.t + B.rim_h / 2.0, DEFAULT))
    s = toward_motor(DEFAULT)
    w0, w1, _, _ = board_window(DEFAULT)
    bb = panel.bounding_box()
    assert (bb.max.Y if s > 0 else -bb.min.Y) == pytest.approx(s * (w0 if s > 0 else w1) - B.gap, abs=1e-6)


@pytest.mark.slow
def test_the_panels_notches(module):
    """The gripper servo's cable's notch - notch_w wide, from notch_reach inside the bay's bore out through the panel's
    edge toward the motor - and the MKS board's near side plug's room: nothing of the panel in either."""
    panel = _leaf(module, PANEL)
    s = toward_motor(DEFAULT)
    for y in (D.bay_r - B.notch_reach + 0.5, D.bay_r, D.bay_r + 1.0):
        assert not is_inside(panel, *back_point(0.0, s * y, B.t / 2.0, DEFAULT)), y
        assert is_inside(panel, *back_point(-(B.notch_w / 2.0 + 0.5), s * (D.bay_r - B.notch_reach + 0.5), B.t / 2.0, DEFAULT))
    for name, (x0, x1, y0, y1, z0, z1) in board_plug_rooms(DEFAULT).items():
        room = Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * Box(x1 - x0, y1 - y0, z1 - z0)
        assert interference(panel, room) < 1e-6, name


@pytest.mark.slow
def test_the_magnets_sit_in_their_pockets_clear_of_the_drive(module, frame):
    """Each disc on its pocket's bottom (touching its own part: the frame's in the step's floor, the panel's in its
    inner face), the pair 0.2 apart across the step's floor; no interference with anything in the drive or the frame."""
    magnets, panel = _leaf(module, MAGNETS), _leaf(module, PANEL)
    frame_depth, panel_depth = back_magnet_depths(DEFAULT)
    for solid in magnets.solids():
        c = solid.center()
        depth = -_out(c) - B.magnet_t / 2.0          # its outer face's depth in from the rear plane
        own = frame if depth > B.t else panel
        assert depth == pytest.approx(frame_depth if own is frame else panel_depth, abs=1e-6)
        assert closest_points(solid, own).distance < 1e-6
    assert interference(magnets, frame) == pytest.approx(0.0, abs=1e-6)
    for other in module.children:
        if other is not magnets:
            assert interference(magnets, other) == pytest.approx(0.0, abs=1e-6), other.label


@pytest.mark.slow
def test_the_panel_clears_the_rest_of_the_drive(module):
    panel = _leaf(module, PANEL)
    for other in module.children:
        if other is not panel:
            assert interference(panel, other) == pytest.approx(0.0, abs=1e-6), other.label
