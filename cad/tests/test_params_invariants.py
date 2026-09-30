"""Locks on lib/params.py - add one whenever a shared dimension is introduced."""
import json
import math
import pathlib

from lib import params as p

MANIFEST = json.loads((pathlib.Path(__file__).resolve().parent.parent / "reference" / "manifest.json").read_text())["parts"]


def test_units():
    assert p.IN == 25.4
    assert 0 < p.NUDGE < 0.1


def test_gt2_belt_drive():
    assert p.GT2_PITCH == 2.0
    assert (p.GT2_PULLEY_90T_TEETH, p.GT2_PULLEY_20T_TEETH) == (90, 20)
    assert math.isclose(p.GT2_RATIO, 4.5)
    assert p.GT2_PULLEY_120T_TEETH == 120 and math.isclose(p.BASE_YAW_RATIO, 6.0)   # the base_yaw belt: 120T on the 20T
    assert math.isclose(p.GT2_PULLEY_90T_PITCH_DIA, 90 * 2 / math.pi)
    # the printed 90T pulley's outer diameter must sit just under its pitch diameter + tooth
    assert p.GT2_PULLEY_90T_PITCH_DIA < MANIFEST["gt2_pulley_90t"]["bbox_size"][0] < p.GT2_PULLEY_90T_PITCH_DIA + 3


def test_gt2_groove():
    # the tooth form (lib/pulley/teeth.py): the fillet smallest, the blend between the bottom and the flank it joins, the
    # flank's centre on the land a fifth of the pitch angle across the groove's centreline (0.8 deg on the 90T), the
    # bottom as deep as the belt's tooth
    assert p.GT2_TIP_R < p.GT2_GROOVE_R < p.GT2_BLEND_R < p.GT2_FLANK_R
    assert p.GT2_FLANK_ANGLE == 360.0 / 5.0
    assert math.isclose(p.flank_offset(90), p.pulley_od(90) / 2.0 * math.sin(math.radians(0.8)))
    assert p.GT2_GROOVE_R < p.GT2_TOOTH_DEPTH < 2.0 * p.GT2_GROOVE_R


def test_gt2_idler():
    # the seller's drawing (lib/belts.py); the belt runs in the channel, on the seat, under the flanges' rims
    assert (p.GT2_IDLER_BORE, p.GT2_IDLER_SEAT_DIA, p.GT2_IDLER_FLANGE_DIA) == (5.0, 12.1, 18.0)
    assert (p.GT2_IDLER_CHANNEL_W, p.GT2_IDLER_WIDTH) == (7.0, 9.0)
    assert p.GT2_BELT_W < p.GT2_IDLER_CHANNEL_W < p.GT2_IDLER_WIDTH
    assert p.GT2_IDLER_BORE < p.GT2_IDLER_SEAT_DIA < p.GT2_IDLER_FLANGE_DIA
    size = MANIFEST["gt2_idler_20t"]["bbox_size"]
    for got, want in zip(size, (p.GT2_IDLER_FLANGE_DIA, p.GT2_IDLER_FLANGE_DIA, p.GT2_IDLER_WIDTH), strict=True):
        assert math.isclose(got, want, abs_tol=0.05)


def test_pancake_envelope_tracks_reference():
    size = MANIFEST["nema17_pancake"]["bbox_size"]
    assert math.isclose(p.PANCAKE_BODY_W, size[0], abs_tol=0.05)
    assert math.isclose(p.PANCAKE_BODY_D, size[1], abs_tol=0.05)
    assert math.isclose(p.PANCAKE_BODY_H, size[2], abs_tol=0.05)
    assert p.PANCAKE_BODY_W < p.NEMA17_FACE + 0.5
    assert p.PANCAKE_MASS_G > 0


def test_belt_motor_and_mks_board_track_reference():
    """The 40 mm kit motor + its MKS SERVO42D board (parts/joints/): envelopes at the split vendor files' bboxes,
    the stack behind a mounting face, and the mounts the SolidWorks links carry (lib/mounts.py)."""
    from lib.datum import BASE_BOTTOM_Y

    motor, board = MANIFEST["nema17_40mm"], MANIFEST["mks_servo42d"]
    assert motor["bbox_size"] == [p.NEMA17_40_BODY_W, p.NEMA17_40_BODY_W + p.NEMA17_40_CONNECTOR_D,
                                  p.NEMA17_40_BODY_LEN + p.NEMA17_40_REAR_STUB_LEN + 22.0]
    assert motor["bbox_min"] == [-p.NEMA17_40_BODY_W / 2, -(p.NEMA17_40_BODY_W / 2 + p.NEMA17_40_CONNECTOR_D),
                                 -(p.NEMA17_40_BODY_LEN + p.NEMA17_40_REAR_STUB_LEN)]
    assert p.NEMA17_40_CONNECTOR_Z0 == -p.NEMA17_40_BODY_LEN and p.NEMA17_40_CONNECTOR_Z0 < p.NEMA17_40_CONNECTOR_Z1 < 0
    assert board["bbox_size"] == [p.MKS_SERVO42D_W, p.MKS_SERVO42D_W, p.MKS_SERVO42D_STACK + p.MKS_SERVO42D_SCREW_REACH]
    assert board["bbox_min"] == [-p.MKS_SERVO42D_W / 2, -p.MKS_SERVO42D_W / 2, -p.MKS_SERVO42D_STACK]
    assert math.isclose(p.MKS_SERVO42D_STACK, 14.1) and p.MKS_SERVO42D_SCREW_REACH < p.NEMA17_40_BODY_LEN
    assert p.NEMA17_40_MASS_G > 0 and p.MKS_SERVO42D_MASS_G > 0
    # the base_yaw motor is the 48 mm one, hanging under the motor mount's 5 mm plate: motor + board (62.1) end
    # BASE_MOTOR_TABLE_CLEAR above the base's bottom face - the face sits under them
    stack = p.CYCLOIDAL_MOTOR_BODY_LEN + p.MKS_SERVO42D_STACK                 # 62.1
    assert math.isclose((p.BASE_MOTOR_PATTERN_CENTRE[1] - stack) - BASE_BOTTOM_Y, p.BASE_MOTOR_TABLE_CLEAR, abs_tol=0.05)
    assert p.BASE_MOTOR_TABLE_CLEAR > 0.0
    # the wrist_pitch motor slides along j2_link's slots to where the stock wrist belt puts it (lib/belts.py), its
    # connector plug clear of the forearm roll's flange wall and its body clear of the wrist boss
    from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, STANDARD_2GT_LENGTHS, closed_belt_length
    from lib.forearm import DEFAULT as FOREARM
    lo, hi = p.J2_MOTOR_SLIDE_RANGE
    assert lo < p.J2_MOTOR_SLIDE_X < hi
    assert p.WRIST_BELT_LENGTH in STANDARD_2GT_LENGTHS
    wrist_x, boss_r = FOREARM.web.wrist_x, FOREARM.boss.dia / 2.0
    assert math.isclose(closed_belt_length(p.J2_MOTOR_SLIDE_X - wrist_x, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH), p.WRIST_BELT_LENGTH, abs_tol=1e-6)
    # over the whole slide: the plug clear of the wall at its wall end, the body clear of the boss at its wrist end
    assert hi + p.NEMA17_40_BODY_W / 2 + p.NEMA17_40_CONNECTOR_D + p.FOREARM_PLUG_CLEARANCE <= p.FOREARM_WALL_X[0] + 1e-9
    assert lo - p.NEMA17_40_BODY_W / 2 >= wrist_x + boss_r
    assert p.FOREARM_WALL_X[0] < p.FOREARM_WALL_X[1] < 0 and p.FOREARM_ROLL_AXIS_Z == 42.0 - 17.0
    assert p.J1_MOTOR_PAD_FACE_Y < 0 < p.J2_MOTOR_WEB_FACE_Z


def test_gripper_rail_tracks_reference():
    size = sorted(MANIFEST["gripper_rail_6mm"]["bbox_size"])
    assert math.isclose(p.RAIL_DIA, size[0], abs_tol=0.01)
    assert math.isclose(p.RAIL_LEN, size[2], abs_tol=0.01)


def test_mg996r_fits_its_reference_envelope():
    size = sorted(MANIFEST["mg996r_servo"]["bbox_size"])
    assert p.MG996R_BODY_W <= size[0] + 0.5
    assert p.MG996R_BODY_L <= size[1] + 0.5
    assert p.MG996R_TAB_L <= size[2] + 1.5
    assert p.MG996R_MASS_G > 0


def test_ky003_hall_sensor_matches_the_listings():
    """The KY-003 module (lib/sensors.py, parts/joints/ky003_hall_sensor): the chip + the board + the pins along X and
    the pins' tails + the board + the header's housing along Z make the listings' 29 x 15 x 7 module, which the
    accepted reference is; the three leads fit the chip and clear the mounting holes, the pins fit the housing, the
    hall element sits inside the chip's body."""
    length = p.KY003_LEAD_OUT + p.A3144_BODY_H + p.KY003_BOARD_L + p.KY003_PIN_OUT
    height = p.KY003_PIN_TAIL + p.KY003_BOARD_T + p.KY003_HEADER_H
    for got, listed in zip((length, p.KY003_BOARD_W, height), (29.0, 15.0, 7.0), strict=True):
        assert abs(got - listed) <= 0.5
    size = MANIFEST["ky003_hall_sensor"]["bbox_size"]
    assert all(math.isclose(a, b, abs_tol=1e-3) for a, b in zip(size, (length, p.KY003_BOARD_W, height), strict=True))
    lead_edge = (p.KY003_PINS - 1) / 2 * p.A3144_LEAD_PITCH + p.A3144_LEAD_W / 2
    assert lead_edge < p.A3144_BODY_W / 2
    assert lead_edge < p.KY003_BOARD_W / 2 - p.KY003_HOLE_INSET - p.KY003_HOLE_DIA / 2    # the leads pass between the holes
    assert (p.KY003_PINS - 1) * p.KY003_PIN_PITCH + p.KY003_PIN_W < p.KY003_PINS * p.KY003_PIN_PITCH
    x, y, z = p.ky003_hall_point()
    assert -(p.KY003_LEAD_OUT + p.A3144_BODY_H) < x < -p.KY003_LEAD_OUT and abs(y) < 1e-9
    assert p.KY003_BOARD_T < z < p.KY003_BOARD_T + p.A3144_BODY_T
    assert p.KY003_MASS_G > 0


# --- cycloidal drive interface (lib/cycloidal re-exported through lib/params.py) --------------------
def test_cycloidal_interface():
    assert p.CYCLOIDAL_RATIO == 20
    assert p.CYCLOIDAL_HOUSING_OD == 129.2
    assert p.CYCLOIDAL_STACK_DEPTH == 60.0
    assert p.CYCLOIDAL_MOTOR_BODY_LEN == 48.0
    assert p.CYCLOIDAL_HUB_OD == 70.3
    assert p.CYCLOIDAL_HUB_PROUD == 5.0
    assert p.CYCLOIDAL_OUTPUT_FACE_Z == 65.0
    assert (p.CYCLOIDAL_ARM_MOUNT_BOLT_CIRCLE, p.CYCLOIDAL_ARM_MOUNT_BOLT_COUNT) == (50.0, 4)
    assert (p.CYCLOIDAL_ARM_MOUNT_ANGLE_OFFSET_DEG, p.CYCLOIDAL_ARM_MOUNT_BOLT_DIA) == (45.0, 4.0)


def test_cycloidal_config_agrees_with_nema17_constants():
    from lib.cycloidal import DEFAULT_CONFIG as cfg

    assert cfg.motor.body_width == p.NEMA17_FACE
    assert cfg.motor.bolt_pattern_square == p.NEMA17_BOLT_SP
    assert cfg.motor.pilot_dia == p.NEMA17_PILOT_DIA
    assert cfg.motor.shaft_dia == p.NEMA17_SHAFT_DIA


def test_cycloidal_stack_positions():
    """The module layout (assemblies/cycloidal_drive.py) - the drive repo's assembly.py numbers."""
    from lib.cycloidal import DEFAULT_CONFIG, stack_positions

    got = stack_positions(DEFAULT_CONFIG)
    expected = {
        "x_disc1": 1.5, "x_disc2": -1.5, "z_motor_plate": 0.0, "z_motor": 0.0, "z_mks_board": -48.0, "z_eccentric_shaft": 0.0,
        "z_ring_gear_body": 9.0, "z_disc1": 13.0, "z_disc2": 25.0, "z_6814_1": 37.0, "z_6814_2": 47.0,
        "z_hub": 37.0, "z_625": 37.0, "z_ring_pins": 5.5, "z_output_pins": 11.0, "z_support_pin": 24.0,
        "z_motor_bolts": -5.0, "z_housing_bolts": 0.5, "z_housing_nuts": 56.0, "hub_top": 65.0,
    }
    assert got.keys() == expected.keys()
    for key, value in expected.items():
        assert math.isclose(got[key], value, abs_tol=1e-9), key


def test_cycloidal_derived_numbers():
    from lib.cycloidal import DEFAULT_CONFIG as cfg
    from lib.cycloidal import (
        hex_circumdiameter,
        hub_height,
        motor_bolt_counterbore_depth,
        ring_pin_engagement,
        ring_pin_hole_depth,
        ring_pin_hole_dia,
    )

    assert cfg.stack_up.bore_zone == 28.0
    assert cfg.stack_up.ring_gear_body_height == 51.0
    assert math.isclose(cfg.housing.lip_bore_dia, 86.15)
    assert math.isclose(cfg.shaft.bridge_flange_od, 23.10)
    assert math.isclose(ring_pin_hole_dia(cfg), 4.20)
    assert ring_pin_engagement(cfg) == 3.5 and ring_pin_hole_depth(cfg) == 31.5
    assert motor_bolt_counterbore_depth(cfg) == 3.0
    assert hub_height(cfg) == 28.0
    assert math.isclose(hex_circumdiameter(7.2), 8.3138, abs_tol=1e-3)
    assert cfg.gear.disc2_phase_deg == -9.0


def test_cycloidal_steel_masses_track_volumes():
    assert math.isclose(p.CYCLOIDAL_RING_PINS_MASS_G, 72.5, abs_tol=0.1)
    assert math.isclose(p.CYCLOIDAL_OUTPUT_PINS_MASS_G, 17.8, abs_tol=0.1)
    assert math.isclose(p.CYCLOIDAL_SUPPORT_PIN_MASS_G, 3.1, abs_tol=0.1)
    assert math.isclose(p.CYCLOIDAL_HOUSING_BOLTS_MASS_G, 39.8, abs_tol=0.1)
    assert math.isclose(p.CYCLOIDAL_HOUSING_NUTS_MASS_G, 6.4, abs_tol=0.1)
    assert math.isclose(p.CYCLOIDAL_MOTOR_BOLTS_MASS_G, 4.3, abs_tol=0.1)
    for mass in (p.CYCLOIDAL_MOTOR_MASS_G, p.BEARING_6003_MASS_G, p.BEARING_6814_MASS_G, p.BEARING_625_MASS_G):
        assert mass > 0


def test_cycloidal_dead_params_gone():
    from lib.cycloidal import DEFAULT_CONFIG as cfg

    for group, name in (
        (cfg.profile, "spline_tolerance"), (cfg.tolerances, "bearing_inner_shaft_sub"),
        (cfg.tolerances, "sliding_clearance_add"), (cfg.housing, "wall_thickness"),
        (cfg.housing, "motor_plate_wall"), (cfg.bearings, "ecc_qty"), (cfg.bearings, "inp_qty"),
    ):
        assert not hasattr(group, name), name


def test_the_6806_pair_fits_every_belt_joint_bore():
    """lib/bearings.py against the three bores it goes in (base_yaw, elbow_pitch, wrist_pitch) and the stubs in it:
    every seat takes the OD and is at least a bearing deep each side of the lip, every lip clears the inner rings,
    every stub is the bore and every shoulder bears on the inner ring only."""
    from lib.base import DEFAULT as BASE
    from lib.coupler import DEFAULT as COUPLER
    from lib.forearm import DEFAULT as FOREARM
    from lib.upper_arm import DEFAULT as UPPER_ARM
    from lib.yaw_coupler import DEFAULT as YAW_COUPLER

    bore, od, width, shoulder = p.BEARING_6806_BORE, p.BEARING_6806_OD, p.BEARING_6806_WIDTH, p.BEARING_6806_SHOULDER_DIA
    assert bore < shoulder < od and p.BEARING_6806_MASS_G > 0
    assert COUPLER.stub_dia == FOREARM.drive.stub_dia == YAW_COUPLER.hub.stub_dia == bore
    assert COUPLER.step[0] == FOREARM.drive.step_dia == shoulder
    e, w, b, c = UPPER_ARM.elbow, FOREARM.web, FOREARM.boss, BASE.bore
    seats = [(e.bore_dia, e.lip_y[1], e.recess_y), (e.seat_dia, e.y0, e.lip_y[0]),              # j1_link: upper, lower
             (b.seat_dia, b.lip_z[1], b.seat_z1), (b.seat_dia, w.z0, b.lip_z[0]),               # j2_link
             (c.upper_dia, c.lip_y[1], BASE.cap.ring_top_y), (c.lower_dia, BASE.cap.boss_y0, c.lip_y[0])]   # base
    for dia, lo, hi in seats:
        assert dia >= od and hi - lo >= width, (dia, lo, hi)
    for lip in (e.lip_dia, b.lip_dia, c.lip_dia):
        assert shoulder + 2.0 < lip < od
