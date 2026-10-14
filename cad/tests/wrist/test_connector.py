"""gripper_j3_connector, parametric (lib/wrist/connector.py): LEGACY is the SolidWorks part - its plate and its boss, two
bodies (the volume / bbox match is tests/test_reference_match.py's) -, DEFAULT what the part builds: the same plate, the
servo holder's and the cover's holes through it, and the hub bolted to the wrist roll pulley's face - its bolt pattern
on the pulley's nut pockets (tests/wrist/test_roll_drive.py), a key's way to every head past the plate, the cable bore."""
import math
from dataclasses import replace

import pytest

from lib.wrist import DEFAULT, LEGACY
from lib.wrist.layout import connector_bolt_points
from tests import built
from tests.helpers import is_inside

C = DEFAULT.connector


@pytest.fixture(scope="module")
def legacy():
    from lib.wrist.connector import build_connector
    return build_connector(LEGACY)


def test_default_is_the_bolted_hub():
    assert not LEGACY.connector.hub and C.hub and DEFAULT.connector == replace(LEGACY.connector, hub=True)
    assert C.hub_flange_dia == DEFAULT.roll.face_dia and C.hub_bore == DEFAULT.roll.bore
    # every bolt's head clear of the core and inside the flange, reached past the plate's flats by a key (Ø2.5 + 0.5)
    head = DEFAULT.roll.clamp_screw.head_dia / 2.0
    for x, z in connector_bolt_points(DEFAULT):
        r = math.hypot(x, z)
        assert r - head >= C.hub_core_dia / 2.0 + 0.5 and r + head <= C.hub_flange_dia / 2.0 + 0.5
        assert abs(x) >= C.half_width + 1.5
    assert C.hub_head_room >= DEFAULT.roll.clamp_screw.head_h + 0.25
    # the screw: through the flange, into its nut under the pulley's face, its tip 2 pitches past
    assert C.bolt_len - C.hub_flange_t >= DEFAULT.roll.face_nut_depth + DEFAULT.roll.nut.h + 2 * 0.5


@pytest.mark.slow
def test_legacy_plate_and_boss(legacy):
    assert len(legacy.solids()) == 2
    plate, boss = sorted(legacy.solids(), key=lambda s: s.bounding_box().min.Y)
    assert (plate.bounding_box().min.Y, plate.bounding_box().max.Y) == pytest.approx((0.0, C.plate_t))
    assert (boss.bounding_box().min.Y, boss.bounding_box().max.Y) == pytest.approx((C.plate_t, C.boss_y1))
    assert not is_inside(boss, 0.0, 8.0, 0.0) and is_inside(boss, C.bore_r_eq + 0.3, 8.0, 0.0)
    assert not is_inside(plate, 0.0, C.plate_t - 0.5, 0.0) and is_inside(plate, 0.0, C.plate_t - 1.5, 0.0)   # the bore's 1 mm recess


@pytest.mark.slow
def test_the_part_is_the_bolted_hub():
    part = built.part("gripper_j3_connector")
    assert part.is_valid and len(part.solids()) == 1
    y_flange, y_end = C.plate_t + C.hub_head_room, C.plate_t + C.hub_len
    bb = part.bounding_box()
    assert (bb.min.Y, bb.max.Y) == pytest.approx((0.0, y_end), abs=1e-6)
    # the plate as SolidWorks drew it, no recess; its 4 holes through
    assert is_inside(part, 0.0, C.plate_t - 0.5, 15.0) and not is_inside(part, C.half_width + 0.5, 1.0, 0.0)
    for x, z, dia in C.holes:
        assert not is_inside(part, x, 1.0, z) and is_inside(part, x + dia / 2.0 + 0.3, 1.0, z)
    # the core and the flange, the cable bore through all, the bolts through the flange, the notches over the holes
    assert is_inside(part, C.hub_core_dia / 2.0 - 0.5, (C.plate_t + y_flange) / 2.0, 0.0)
    assert not is_inside(part, C.hub_core_dia / 2.0 + 0.5, (C.plate_t + y_flange) / 2.0, 0.0)
    assert is_inside(part, C.hub_flange_dia / 2.0 - 0.5, (y_flange + y_end) / 2.0, 0.0)
    for y in (1.0, (C.plate_t + y_flange) / 2.0, (y_flange + y_end) / 2.0):
        assert not is_inside(part, 0.0, y, 0.0) and not is_inside(part, C.hub_bore / 2.0 - 0.3, y, 0.0)
    for x, z in connector_bolt_points(DEFAULT):
        assert not is_inside(part, x, (y_flange + y_end) / 2.0, z)
    for x, z, _ in C.holes:
        assert not is_inside(part, x, (y_flange + y_end) / 2.0, z)
