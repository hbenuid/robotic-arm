"""The wrist body's and the wrist roll drive's derived points (pure math, no kernel): what lib/wrist/link.py,
lib/wrist/roll.py, the drive's module table and the tests share."""
from __future__ import annotations

import math

from lib.belts import GT2_PULLEY_20T_TEETH, pulley_circle
from lib.datum import compose
from lib.fasteners import M3_PITCH
from lib.motors import MKS_PLUG_ROOM, MKS_SERVO42D_HEADERS
from lib.wrist.params import DEFAULT, WristConfig

_AXES_DEG = (0.0, 90.0, 180.0, 270.0)


def seat_bolt_points(cfg: WristConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """(x, y, angle_deg) of the coupler flange's 4 bolts: on the axes about the pitch axis at SeatParams.bolt_r."""
    px, r = cfg.plate.pitch_x, cfg.seat.bolt_r
    return [(px + r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), a) for a in _AXES_DEG]


def pulley_bolt_points(cfg: WristConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, y) of the wrist 90T's 4 M4 through the flange (FlangeParams): on the axes about the pitch axis at pulley_bolt_r."""
    px, r = cfg.plate.pitch_x, cfg.flange.pulley_bolt_r
    return [(px + r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in _AXES_DEG]


def end_face_holes(cfg: WristConfig = DEFAULT) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """((y, z) of the M3 - 2 per row in m3_rows -, (y, z) of the 4 M4) on the end face, about the roll axis."""
    e = cfg.end_face
    h = e.m3_sp / 2.0
    m3 = [(sy * h, e.axis_z + sz * h) for sy in (-1.0, 1.0) for sz in e.m3_rows]
    m4 = [(sy * e.m4_y, e.axis_z + sz * e.m4_dz) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
    return m3, m4


def slope_x(z: float, cfg: WristConfig = DEFAULT) -> float:
    """x of the slope plane at height z (the wedge under it is x >= slope_x(z))."""
    t = cfg.tower
    return t.slope_x0 + (z - cfg.plate.thickness) * (t.slope_x1 - t.slope_x0) / (t.z1 - cfg.plate.thickness)


# --- the wrist roll drive (WristRollParams; its module frame: origin on R' at the pitch axis' station, +Z along R'
# toward the gripper, +X away from the plate, +Y = wrist_link's +Y - R' `side` of the motor) ---------------------------

def module_frame_in_host(cfg: WristConfig = DEFAULT) -> tuple:
    """The drive's module frame as data in wrist_link's part frame (lib/mounts.py ModuleMount): origin on R' -
    side x centre_distance along Y from the pitch axis, axis_h over the plate's coupler face (part z thickness) -,
    module +Z = part +X, +X = part -Z, +Y = part +Y."""
    d = cfg.roll
    return ((cfg.plate.pitch_x, d.side * d.centre_distance, cfg.plate.thickness - d.axis_h), (0.0, 90.0, 0.0))


def stack_positions(cfg: WristConfig = DEFAULT) -> dict[str, float]:
    """Module-frame stations of the drive (the frame, the pulley and the shaft builders emit their geometry at these
    stations, their rows at 0). Along Z: the pancake centred on the pitch axis (its body's middle at z 0), its 20T's
    band where the 40T's is; from the band back: the band's rear flange, run_gap, bearing 2 (the boss's top), the lip,
    bearing 1, the shaft behind it - its rear face at the rotor clamp's tips -, the bay; from the band forward the face
    the connector bolts to. Along X: the plate's coupler face and free face under R'. Every number a row or a test
    needs comes from here."""
    d, p = cfg.roll, cfg.roll.pancake
    z_motor_face = p.body_length / 2.0                            # 11.5: the body centred on the pitch axis
    z_ring_mid = z_motor_face + d.t20                             # 23.95: the 20T's band = the 40T's
    z_ring = z_ring_mid - d.ring_width / 2.0                      # 20.45
    z_ring_flange_1 = z_ring - d.ring_flange_t                    # 19.25: the band's rear flange
    z_boss = z_ring_flange_1 - d.run_gap                          # 18.25: the bearing boss's top = bearing 2's front face
    z_bearing_2 = z_boss - d.bearing_width                       # 11.25
    z_lip = z_bearing_2 - d.lip                                   # 9.25
    z_bearing_1 = z_lip - d.bearing_width                        # 2.25: the tower's rear face, in the bay
    z_meet = z_lip + d.lip / 2.0                                  # 10.25: the two hubs meet in the lip's middle
    z_ring_end = z_ring + d.ring_width + d.ring_flange_t          # 28.65: the band's front flange
    z_face = z_ring_end + d.face_t                                # 34.65: the pulley's face (the connector's seat)
    z_clamp_head = z_face - d.clamp_screw.head_h - d.clamp_head_clear   # 31.4: the clamp's counterbores' floor
    z_shaft_end = z_clamp_head - d.clamp_screw_len               # -3.6: the clamp's tips = the shaft's rear face
    z_clamp_nut = z_shaft_end + 2 * M3_PITCH                      # -2.6: each clamp nut's far face
    z_bay = z_shaft_end - (z_meet - z_bearing_1) - d.bay_clear    # -12.6: the bay's floor - room to slide the shaft in
    z_motor_rear = z_motor_face - p.body_length                   # -11.5
    z_pocket_rear = z_motor_rear - d.motor_drop - d.pocket_rear   # -13.75
    z_face_nut = z_face - d.face_nut_depth - d.nut.h              # 31.25: each connector nut's far face
    x_coupler = -d.axis_h                                         # -27: the plate's coupler face
    x_free = cfg.plate.thickness - d.axis_h                       # -22: its free face, the pocket's floor
    return {
        "z_pulley": 0.0, "z_shaft": 0.0,
        "z_rear": z_bay - d.tower_wall,                           # -14.6: the frame's rear, the tower's behind the bay (what the
        #                                                           wrist pitch's lower limit first meets); behind the pocket the
        #                                                           board's window (board_window) - no wall
        "z_board_back": z_motor_rear - d.board_stack,             # -25.6: the MKS board's back, 11 behind the frame's rear
        "z_bay": z_bay, "z_shaft_end": z_shaft_end, "z_clamp_nut": z_clamp_nut,
        "z_clamp_nut_seat": z_clamp_nut + d.nut.h,                # -0.2: its bearing face, toward the pulley
        "z_stop_lug": z_shaft_end,                                # -3.6 .. +stop_lug_t on the shaft's flange
        "z_stop_post": z_bearing_1 - d.stop_post_t,               # -1.6 .. 2.25 on the tower's rear face
        "z_shoulder_1": z_bearing_1 - d.run_gap,                  # 1.25: the shaft's Ø33 shoulder on bearing 1's inner ring
        "z_bearing_1": z_bearing_1, "z_lip": z_lip, "z_meet": z_meet, "z_bearing_2": z_bearing_2, "z_boss": z_boss,
        "z_ring_flange_1": z_ring_flange_1, "z_ring": z_ring, "z_ring_mid": z_ring_mid, "z_ring_end": z_ring_end,
        "z_face": z_face, "z_clamp_head": z_clamp_head, "z_face_nut": z_face_nut,
        "z_face_nut_seat": z_face_nut + d.nut.h,                  # 33.65: its bearing face, toward the connector
        "z_connector": z_face + cfg.connector.hub_len,            # 41.9: the connector plate's hub-side face
        "z_motor_face": z_motor_face, "z_motor_rear": z_motor_rear, "z_pocket_rear": z_pocket_rear,
        "z_front": z_motor_face + d.pad_t,                        # 15.5: the plate's front face = the frame's front face
        "z_20t": z_motor_face + p.pilot_height + d.pulley_seat,   # 13.0: the 20T's hub face
        "x_coupler": x_coupler, "x_free": x_free,
        "x_motor": 0.0, "y_motor": -d.side * d.centre_distance,   # the pancake on the pitch axis
    }


def motor_pocket(cfg: WristConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(y0, y1, z0, z1) of the pancake's body + the tension travel either way + pocket_clear; from behind its rear face
    (motor_drop + pocket_rear) to its face. The pocket in the frame runs on from its edge toward R' down out through the
    bottom (the frame's end beyond the motor, away from R': the pancake slides in from there), from the free face to the
    side wall's inside (side_wall_x)."""
    d, S = cfg.roll, stack_positions(cfg)
    half = d.pancake.body_w / 2.0 + d.pad_slot_len / 2.0 + d.pocket_clear
    return (S["y_motor"] - half, S["y_motor"] + half, S["z_pocket_rear"], S["z_motor_face"])


def toward_motor(cfg: WristConfig = DEFAULT) -> float:
    """+1 or -1: the module's y direction from R' to the pitch axis (the motor) and on to the frame's bottom."""
    return -cfg.roll.side


def side_wall_x(cfg: WristConfig = DEFAULT) -> float:
    """The closed far face's wall's inside (module x): the pocket and the board's window stop there."""
    return cfg.roll.frame_r - cfg.roll.side_t


def board_window(cfg: WristConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(y0, y1, z0, z1) of the MKS board's window behind the pocket: the board's housing square + the tension travel
    either way + board_clear; from the frame's rear (z0: the cut runs on out through it) to board_clear in front of the
    pancake's rear face, where the housing starts. Like the pocket it runs on down out through the bottom, from the free
    face to the side wall."""
    d, S = cfg.roll, stack_positions(cfg)
    half = d.board_w / 2.0 + d.pad_slot_len / 2.0 + d.board_clear
    return (S["y_motor"] - half, S["y_motor"] + half, S["z_rear"], S["z_motor_rear"] + d.board_clear)


def rear_line(cfg: WristConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The frame's slanted rear seen along the pitch axis, as two (y, z) points: from the tower's far rear corner (-s x
    frame_r, z_rear) down to rear_low at rear_bend toward the motor (s = toward_motor); flat at rear_low beyond. The back
    panel lies on it."""
    d, S, s = cfg.roll, stack_positions(cfg), toward_motor(cfg)
    return (-s * d.frame_r, S["z_rear"]), (s * d.rear_bend, d.rear_low)


def rear_plane(cfg: WristConfig = DEFAULT) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(point, outward normal) of the rear's slanted plane in the module frame (it holds the module's x direction)."""
    (ya, za), (yb, zb) = rear_line(cfg)
    dy, dz = yb - ya, zb - za
    n = math.hypot(dy, dz)
    ny, nz = dz / n, -dy / n
    if nz > 0.0:
        ny, nz = -ny, -nz
    return (0.0, ya, za), (0.0, ny, nz)


def rear_z(y: float, cfg: WristConfig = DEFAULT) -> float:
    """The frame's rear (least module z) at y, between the tower's far side and the octagon: the slanted plane, then flat."""
    (ya, za), (yb, zb) = rear_line(cfg)
    t = (y - ya) / (yb - ya)
    return za + (zb - za) * min(max(t, 0.0), 1.0)


def pancake_plug_groove(cfg: WristConfig = DEFAULT) -> tuple[float, float, float, float, float, float]:
    """(x0, x1, y0, y1, z0, z1) of the groove inside the side wall for the pancake's plug (its boss plug_d proud of the body
    toward +X: motor_spin_deg): plug_clear round the boss and along Y past the tension travel, from the plug on down out
    through the bottom (it slides in with the motor) - y1 the bottom's side, open-ended - and from plug_clear over the
    boss on down out through the rear (its lead to the board): z0 open-ended."""
    d, S, s = cfg.roll, stack_positions(cfg), toward_motor(cfg)
    p = d.pancake
    x1 = S["x_motor"] + p.body_w / 2.0 + p.plug_d + d.plug_clear
    near = S["y_motor"] - s * (p.plug_w / 2.0 + d.pad_slot_len / 2.0 + d.plug_clear)
    z1 = S["z_motor_rear"] + p.plug_h + d.plug_clear
    return side_wall_x(cfg), x1, near, s * math.inf, -math.inf, z1


def board_plug_rooms(cfg: WristConfig = DEFAULT) -> dict[str, tuple[float, float, float, float, float, float]]:
    """name -> (x0, x1, y0, y1, z0, z1) in the module frame of each of the MKS board's three plugs' rooms (lib/motors.py:
    each header's outline carried MKS_PLUG_ROOM straight out from its face - tests/test_sweeps.py _plug_rooms), the
    board on the pancake's rear face turned with it (motor_spin_deg)."""
    d, S = cfg.roll, stack_positions(cfg)
    a = math.radians(d.motor_spin_deg)
    c, sn = round(math.cos(a), 12), round(math.sin(a), 12)
    rooms = {}
    for name, ((x0, y0, z0, x1, y1, z1), (ux, uy)) in MKS_SERVO42D_HEADERS.items():
        r = MKS_PLUG_ROOM[name]
        xa, xb = ((x1, x1 + r) if ux > 0 else (x0 - r, x0)) if ux else (x0, x1)
        ya, yb = ((y1, y1 + r) if uy > 0 else (y0 - r, y0)) if uy else (y0, y1)
        xs = [S["x_motor"] + x * c - y * sn for x in (xa, xb) for y in (ya, yb)]
        ys = [S["y_motor"] + x * sn + y * c for x in (xa, xb) for y in (ya, yb)]
        rooms[name] = (min(xs), max(xs), min(ys), max(ys), S["z_motor_rear"] + z0, S["z_motor_rear"] + z1)
    return rooms


def pad_bolt_points(cfg: WristConfig = DEFAULT) -> list[tuple[float, float]]:
    """The plate's 4 bolt slots' centres in the module x-y plane: the pancake's bolt_sp square about its axis."""
    d, S = cfg.roll, stack_positions(cfg)
    h = d.pancake.bolt_sp / 2.0
    return [(S["x_motor"] + sx * h, S["y_motor"] + sy * h) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]


def pancake_location(cfg: WristConfig = DEFAULT) -> tuple:
    """The pancake's row in the module (its vendor frame as data): its axis on (x_motor, y_motor) along +Z, its mounting
    face on the plate's rear face, spun motor_spin_deg about its axis (its plug, the vendor frame's -Y, toward +X)."""
    d, S = cfg.roll, stack_positions(cfg)
    a = math.radians(d.motor_spin_deg)
    p = d.pancake
    return ((S["x_motor"] + p.axis_y * math.sin(a), S["y_motor"] - p.axis_y * math.cos(a), S["z_motor_face"] - p.face_z),
            (0.0, 0.0, d.motor_spin_deg))


def roll_belt_loop(cfg: WristConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """The wrist roll belt's loop (lib/belts.py) in the belt's own frame: the pancake's 20T on the origin, the pulley's
    40T band on +X, centre_distance on (the stock belt at the motor slots' middle - no idler). The belt's row turns +X
    toward R' (the module's +Y times side)."""
    d = cfg.roll
    return [pulley_circle(0.0, 0.0, GT2_PULLEY_20T_TEETH), pulley_circle(d.centre_distance, 0.0, d.ring_teeth)]


def clamp_points(cfg: WristConfig = DEFAULT) -> list[tuple[float, float]]:
    """The rotor clamp's 4x M3 in the module x-y plane: on clamp_circle_dia, from clamp_angle_deg."""
    d = cfg.roll
    r = d.clamp_circle_dia / 2.0
    return [(r * math.cos(math.radians(d.clamp_angle_deg + 90.0 * k)), r * math.sin(math.radians(d.clamp_angle_deg + 90.0 * k)))
            for k in range(4)]


def clamp_nut_pocket(cfg: WristConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(r_in, r_out, z0, z1) of each clamp nut's pocket in the shaft's flange, radially along its screw's angle: from
    inside the cable bore past the nut's corner by nut_fit; from the shaft's rear face (open: the nut goes in there) to
    the nut's bearing face."""
    d, S = cfg.roll, stack_positions(cfg)
    r_out = d.clamp_circle_dia / 2.0 + d.nut.af / math.sqrt(3.0) + d.nut_fit
    return d.bore / 2.0 - 1.0, r_out, S["z_shaft_end"], S["z_clamp_nut_seat"]


def gripper_frame_in_module(cfg: WristConfig = DEFAULT) -> tuple:
    """The gripper module's frame (assemblies/gripper.py, lib/gripper/layout.py) as data in this drive's module frame: on
    R' at gripper_j3_connector's plate's gripper face (z_connector + its plate), turned with the connector's plate - the
    gripper's +Y (the jaws' travel) the plate's -z, at connector_turn_deg - 180 about R'."""
    return ((0.0, 0.0, stack_positions(cfg)["z_connector"] + cfg.connector.plate_t), (0.0, 0.0, cfg.roll.connector_turn_deg - 180.0))


def gripper_frame_in_host(cfg: WristConfig = DEFAULT) -> tuple:
    """The gripper module's frame as data in wrist_link's part frame (lib/mounts.py ModuleMount "gripper#1")."""
    return compose(module_frame_in_host(cfg), gripper_frame_in_module(cfg))


def connector_bolt_points(cfg: WristConfig = DEFAULT) -> list[tuple[float, float]]:
    """The connector's 4x M3 in its plate's (x, z) (gripper_j3_connector's frame): on bolt_circle_dia at bolt_deg."""
    c = cfg.connector
    r = c.bolt_circle_dia / 2.0
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in c.bolt_deg]


def connector_bolt_points_module(cfg: WristConfig = DEFAULT) -> list[tuple[float, float]]:
    """The same 4 bolts in the module x-y plane (the pulley's face): the connector's plate turned connector_turn_deg
    about R' (its (x, z) plate axes a rotation of the module's (x, y))."""
    a = math.radians(cfg.roll.connector_turn_deg)
    return [(x * math.cos(a) - z * math.sin(a), x * math.sin(a) + z * math.cos(a)) for x, z in connector_bolt_points(cfg)]


def face_nut_pocket(cfg: WristConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(r_in, r_out, z0, z1) of each connector nut's pocket in the pulley's face, radially along its bolt's angle: from
    the nut's inner corner (less nut_fit) out through the face's rim; from nut_fit under the nut to its bearing face."""
    d, c, S = cfg.roll, cfg.connector, stack_positions(cfg)
    r_in = c.bolt_circle_dia / 2.0 - d.nut.af / math.sqrt(3.0) - d.nut_fit
    return r_in, d.face_dia / 2.0 + 1.0, S["z_face_nut"] - d.nut_fit, S["z_face_nut_seat"]
