"""ForearmConfig - every dimension of the forearm: j2_link (the web between the elbow and the wrist pivots),
j2_cap_1 (the lid over the motor side) and j2_cap_2 (the belt tray), all in j2_link's part frame.

Frame (= the SolidWorks part frame of j2_link, which placements.json places): origin on the elbow pivot, the
wrist pivot at x = wrist_x (-210), +Z = N (the pitch-axis direction: the motor-body / cap-1 side; -Z = the
belt tray / cap-2 side; the upper arm lies at z < -8.5). The two caps are modelled in THIS frame too - their
SolidWorks part frames differ (parts/joints/j2_cap_*.py LOCAL_FROM_REF).

Two configurations: LEGACY reproduces the three SolidWorks references exactly (the parts' REFERENCE_BUILD -
tests/test_reference_match.py), DEFAULT is what the parts build. In M1 they are the same; the forearm roll
(M2) makes DEFAULT end the forearm at a flange wall instead of the elbow disc.

Every number below was measured on the references 2026-09-22 (planar / cylindrical face census;
tests/forearm/test_forearm_legacy.py re-checks the builds against them): [REFERENCE] unless tagged.
Units mm, degrees where named *_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, centre_distance
from lib.cycloidal.params import MotorParams
from lib.motors import MOTOR_40, NEMA17_40_BODY_LEN


@dataclass(frozen=True)
class WebParams:
    """The 11 mm web: a stadium between the two pivots (r = half_w at both ends)."""

    z0: float = 8.0            # bottom face (the belt-tray side, j2_cap_2's rim sits on it)
    z1: float = 19.0           # top face (the motor's mounting face - lib/params.py J2_MOTOR_WEB_FACE_Z)
    half_w: float = 45.0       # +/- y
    wrist_x: float = -210.0    # the wrist_pitch pivot (the elbow pivot is the origin)

    @property
    def thickness(self) -> float:
        return self.z1 - self.z0


@dataclass(frozen=True)
class ElbowDiscParams:
    """The elbow end: a disc under the web whose z=0 face bolts to j3_coupler#1 (4x M4 into captive nuts
    dropped in from the top). The forearm roll's elbow block carries the same interface (M3)."""

    dia: float = 90.0
    z0: float = 0.0
    z1: float = 19.0           # = the web's top face
    bore_dia: float = 54.89    # the coupler's hub passes through
    bolt_dia: float = 4.1      # 4x M4 clearance
    bolt_r: float = 35.0       # on the axes (0 / 90 / 180 / 270 deg)
    bolt_z1: float = 15.9      # the clearance hole stops where the nut pocket begins
    nut_af: float = 6.86       # hex pocket across flats, a vertex toward the disc centre
    nut_z0: float = 15.9       # pocket from here to the top face (3.1 deep)


@dataclass(frozen=True)
class WristBossParams:
    """The wrist end: a boss above the web with the wrist_pitch bearing seat (Ø42.2, split by a lip) and an
    Ø80 recess on top (the wrist 90T pulley / j3_coupler#2 side)."""

    dia: float = 90.0
    z1: float = 33.5
    seat_dia: float = 42.2
    seat_z1: float = 27.5
    lip_dia: float = 37.65
    lip_z: tuple = (17.0, 19.0)
    recess_dia: float = 80.0
    recess_z0: float = 27.5


@dataclass(frozen=True)
class SlotParams:
    """The wrist-pitch motor's slide through the web: a central slot for the pilot boss and two side slots for
    the bolt pattern (31 mm square -> +/- 15.5, the slots at +/- 15.4). Arc centres in x."""

    centre_w: float = 20.0
    centre_x: tuple = (-148.0, -56.0)
    side_w: float = 3.2
    side_y: float = 15.4
    side_x: tuple = (-157.0, -47.0)


@dataclass(frozen=True)
class SocketParams:
    """Blind Ø5.18 x 2 locating sockets (dowel / magnet seats) on a grid the caps mirror, plus a pair under
    the wrist boss. The web carries the grid on both faces."""

    dia: float = 5.18
    depth: float = 2.0
    grid_x: tuple = (-30.0, -75.0, -120.0, -165.0)
    grid_y: float = 40.0
    wrist: tuple = ((-244.64, 20.0), (-244.64, -20.0))   # bottom face only (web + cap 2)


@dataclass(frozen=True)
class Cap1Params:
    """j2_cap_1 - the lid over the web's top face: a rim on the web (z0) under a 1.5 lid, a pocket between,
    the motor window through the lid. Outline: the disc's r at the elbow, flat sides, a concave arc round the
    wrist boss (outer_wrist_r) - it stops where the boss begins."""

    z0: float = 19.0           # rim face (= the web's top face)
    z1: float = 33.5           # lid outer face
    lid: float = 1.5
    pocket_half_w: float = 35.0
    pocket_elbow_r: float = 35.0     # the pocket wraps the elbow axis (its wall is an arc about it) ...
    pocket_wrist_r: float = 55.0     # ... and stops at an arc about the wrist axis
    outer_wrist_r: float = 46.0
    window_len: float = 60.0         # the motor window, window_len long centred window_offset from the motor axis
    window_offset: float = 0.0       # (ForearmConfig.motor_x; DEFAULT: past the body and the connector by 2 mm)
    window_half_w: float = 24.0


@dataclass(frozen=True)
class Cap2Params:
    """j2_cap_2 - the belt tray under the web: a rim on the web's bottom face (z1) over a 1.5 floor, the
    pocket between, OPEN toward the elbow (only the floor lip closes it there); an arc channel about the elbow
    axis cut 10 deep from the outer face through floor and rims. Outline: the boss's r at the wrist, flat
    sides, a concave arc round the elbow (outer_elbow_r)."""

    z0: float = -5.5           # outer (floor) face
    z1: float = 8.0            # rim face (= the web's bottom face)
    floor: float = 1.5
    pocket_half_w: float = 35.0
    pocket_wrist_r: float = 35.0     # the pocket wraps the wrist axis
    outer_elbow_r: float = 55.0
    channel_r: tuple = (70.75, 93.25)   # about the elbow axis [REFERENCE] - purpose unknown (it clears nothing of j1_link)
    channel_depth: float = 10.0         # from the outer face


@dataclass(frozen=True)
class RollEndParams:
    """The forearm roll's rotor side (roll=True): the forearm ends at a flange wall instead of the elbow disc.
    The roll axis runs along -X through (y 0, z axis_z) - the wrist centre, where the wrist_pitch and wrist_roll
    axes meet (42 − 17 along N from the elbow origin), so the three wrist axes stay concurrent. The rotor (the
    hollow roll shaft, parts/joints/forearm_roll_shaft) bolts its Ø60 flange into a shallow locating recess on the
    wall's elbow face; the cables pass through the bore. Elbow block + shaft: lib/forearm/roll.py (M3)."""

    axis_z: float = 25.0                  # [REFERENCE] the wrist centre's N-station above the elbow origin (42 − 17)
    wall_x: tuple = (-96.0, -88.0)        # [DESIGN] the wall's wrist face .. elbow face (8 thick)
    wall_z: tuple = (-10.0, 60.0)         # [DESIGN] full width y +/- half_w; below the tray, above the lid - nothing swings there
    flange_dia: float = 39.7              # [DESIGN] the shaft's end spigot (< the 6808 bore: bearing 2 slides over it from the wrist end)
    flange_recess_add: float = 0.3        # [DESIGN] PETG mating clearance on the recess diameter (Ø40.0)
    flange_recess_depth: float = 2.0      # [DESIGN] a locating spigot, not a load path
    bolt_circle_dia: float = 32.0         # [DESIGN] 4x M3 on the axes (0 / 90 / 180 / 270 about the roll axis), into the shaft's end wall
    bolt_count: int = 4
    bolt_angle_deg: float = 0.0
    bolt_dia: float = 3.4                 # [DESIGN] M3 clearance through the wall (heads on the wrist face; self-tapping in the shaft's end)
    cable_bore: float = 24.0              # [DESIGN] = the shaft's bore (the shaft's Ø40 end leaves 8 mm of wall for the bolts)
    plug_clearance: float = 10.0          # [DESIGN] the wrist motor's connector plug needs this much room to the wall
    wrist_belt: int = 264                 # [ESTIMATE] 264-2GT closed belt, 6 mm: sets the motor slide (motor_x)


@dataclass(frozen=True)
class RollDriveParams:
    """The forearm roll drive (assemblies/forearm_roll_drive.py): the elbow block (stator - bolted to j3_coupler#1
    through the SolidWorks disc's interface; a Ø housing_od housing round the roll axis with bearing 1's seat, the
    cavity the shaft's integral 90T ring runs in, the belt window in its +Y wall, the cable window in its +X wall,
    two lugs for the end cap and the motor pad tower), the hollow roll shaft (rotor - journals either side of the
    ring, a neck bearing 2 slides over, the Ø39.7 end spigot the forearm's wall bolts onto), the bolt-on END CAP
    carrying bearing 2's seat and lip, the 40 mm kit motor UP in the swing plane (+Y) with its 20T and a belt to the
    ring. MODULE FRAME: origin at the roll axis' crossing with the elbow axis (host (0, 0, axis_z)); +Z along the roll
    axis toward the wrist (host −X); +X = host +Z (N); +Y = host +Y = up in the arm's swing plane. Stations in that
    frame are layout.stack_positions(). Assembly: bearing 1 onto the shaft's elbow journal, the shaft in from the
    wrist end through the cavity to the lip, bearing 2 over the neck onto its journal, the cap over it (2x M3 into the
    lugs), the forearm wall onto the spigot (4x M3 into the shaft's end)."""

    # bearings: 2x 6808-2RS - bearing 1 in the housing's seat against its lip, bearing 2 in the cap's seat against the cap's lip
    bearing_bore: float = 40.0            # [DATASHEET] 6808-2RS (61808)
    bearing_od: float = 52.0
    bearing_width: float = 7.0
    seat_add: float = 0.15                # [DESIGN] PETG press allowance on the seat diameter (cf. the drive's 6814 seat)
    journal_add: float = 0.3              # [DESIGN] the printed journal's interference in the inner race (cf. the drive's hub)
    inner_race_od: float = 44.5           # [ESTIMATE] 6808 inner-race outer edge - the shaft's Ø44 shoulders must not touch the outer race
    # the housing (stator)
    housing_od: float = 70.0              # [DESIGN] round the cavity; its underside carries a flat (flat_x) above the upper arm's slab
    z_end: float = 40.0                   # [DESIGN] the closed elbow end (>= 1 mm past the disc's M4 nut pocket, clear of j3_coupler)
    end_wall: float = 3.0                 # [DESIGN] 40..43
    lip: float = 3.0                      # [DESIGN] 43..46, ID lip_id: bearing 1's outer race stops on it
    lip_id: float = 46.0
    cavity_dia: float = 62.0              # [DESIGN] round the ring's flanges (+1.4), leaving 2 mm of wall under the flat
    ring_gap: float = 3.8                 # [DESIGN] a bearing's face .. the ring's flange (the shaft's Ø44 shoulders)
    flat_x: float = -33.0                 # [DESIGN] the housing's and the cap's underside (module -X = host -Z): host z -8, 0.5 mm above the upper arm's slab (-8.5)
    cable_window_w: float = 20.0          # [DESIGN] the bore's exit through the +X wall (the top face side), z_end .. z_end + cable_window_len
    cable_window_len: float = 6.0
    belt_window_half_x: float = 22.0      # [DESIGN] the belt's two runs cross the +Y wall at |x| <= 20 (tangent points on the ring at +/-26)
    belt_window_y: tuple = (22.0, 36.0)   # [DESIGN] from inside the cavity out through the wall
    belt_window_margin: float = 1.0       # [DESIGN] past the ring's flanges, both sides
    lug_y: float = 38.5                   # [DESIGN] the cap's 2x M3: at (+lug_y, 0) - the top face side - and (0, -lug_y) - below in the swing plane; never -X (the slab)
    lug_w: float = 7.0
    lug_len: float = 12.0
    lug_root: float = 33.0                # [DESIGN] the lug / ear boxes start inside the wall (r 31..35)
    cap_bolt_dia: float = 3.4             # [DESIGN] through the cap's ears
    cap_tap_dia: float = 2.5              # [DESIGN] M3 self-tapping in the housing's lugs
    cap_tap_depth: float = 8.0
    # the end cap
    cap_lip: float = 2.0                  # [DESIGN] past bearing 2's seat, ID lip_id
    # the shaft (rotor)
    bore: float = 24.0                    # [DESIGN] the cable bore (= the wall's cable_bore)
    shoulder_od: float = 44.0             # [DESIGN] < inner_race_od
    neck_od: float = 38.0                 # [DESIGN] beyond journal 2 to the spigot: bearing 2 slides over it (< bearing_bore)
    shaft_end_clear: float = 0.5          # [DESIGN] the shaft's elbow end .. the housing's end wall
    end_bolt_tap_dia: float = 2.5         # [DESIGN] 4x M3 self-tapping into the shaft's end wall (RollEndParams.bolt_circle_dia)
    end_bolt_depth: float = 8.0
    stop_lug_r: tuple = (17.0, 28.0)      # [DESIGN] the rotor's hard-stop lug on the neck (rooted 2 mm inside its Ø38), at +X in the zero pose
    stop_post_r: tuple = (24.0, 30.0)     # [DESIGN] the cap's post on its outer face, at -X: they overlap r 24..28
    stop_t: float = 3.0                   # [DESIGN] both lugs; the lug overlaps the post by 1 mm along Z
    stop_deg_width: float = 10.0          # [DESIGN] angular width of each: contact at +/- (180 - width) = +/- stop_deg
    stop_deg: float = 170.0               # [ESTIMATE] = FOREARM_ROLL_LIMIT_DEG
    # the 90T ring (integral; the 20T is flanged on its hub side only, so the ring carries two flanges)
    ring_teeth: int = 90
    ring_width: float = 7.0               # [DATASHEET] 6 mm belt
    ring_flange_dia: float = 59.19        # [REFERENCE] the SolidWorks 90T's flanges
    ring_flange_t: float = 1.2
    # the motor: up in the swing plane (+Y), lifted motor_x along +X so its body clears the elbow disc's top face;
    # body toward the elbow (-Z), shaft toward the wrist; slotted along the axis -> motor direction for tension
    motor: MotorParams = MOTOR_40
    motor_x: float = 18.0                 # [DESIGN] >= (disc z1 + 1 + body_w / 2) - axis_z = 16: the body 1 mm above the disc's face
    roll_belt: int = 240                  # [ESTIMATE] 240-2GT closed belt, 6 mm: sets the centre distance (60.9) and so motor_y
    t20_hub: float = 10.95                # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    pulley_lift: float = 0.5              # [DESIGN] the 20T's hub face above the pad plate's top
    pad_t: float = 3.0                    # [DESIGN] the motor pad plate (on the shaft side of the mounting face; the Ø22 x 2 pilot boss centres in it)
    pad_w: float = 46.0                   # [DESIGN] square, centred on the motor axis
    pad_slot_len: float = 5.0             # [DESIGN] +/- 2.5 belt-tension slide along the axis -> motor direction
    pad_bolt_dia: float = 3.4
    pad_pilot_w: float = 22.3             # [DESIGN] the pilot boss slot
    pad_root_y: float = 33.0              # [DESIGN] the filler between the housing's wall and the plate starts inside the wall

    @property
    def t20(self) -> float:
        """The 20T's tooth-band centre from the motor's mounting face: through the pad plate, the lift, the hub."""
        return self.pad_t + self.pulley_lift + self.t20_hub

    @property
    def centre_distance(self) -> float:
        """Ring axis .. motor axis: what the roll belt sets."""
        return centre_distance(self.roll_belt, self.ring_teeth, GT2_PULLEY_20T_TEETH)

    @property
    def motor_y(self) -> float:
        """The motor axis' +Y offset: the belt's centre distance less the lift along X."""
        return (self.centre_distance ** 2 - self.motor_x ** 2) ** 0.5


@dataclass(frozen=True)
class ForearmConfig:
    roll: bool = False                     # end the forearm at the roll flange wall instead of the elbow disc
    motor_x: float = -118.0                # the wrist-pitch motor's axis on the slide [ESTIMATE] (lib/params.py J2_MOTOR_SLIDE_X)
    slide_range: tuple = (-141.5, -62.5)   # motor-axis x the slots allow (bolt pattern inside the side slots)
    web: WebParams = WebParams()
    disc: ElbowDiscParams = ElbowDiscParams()
    boss: WristBossParams = WristBossParams()
    slot: SlotParams = SlotParams()
    sockets: SocketParams = SocketParams()
    cap1: Cap1Params = Cap1Params()
    cap2: Cap2Params = Cap2Params()
    roll_end: RollEndParams = RollEndParams()
    drive: RollDriveParams = RollDriveParams()


LEGACY = ForearmConfig()     # the three SolidWorks parts, exactly

# The forearm with the roll joint: the wall replaces the elbow disc; the wrist-pitch motor slides toward the wrist
# so its connector plug clears the wall - its position is what a stock 264-2GT belt sets (lib/belts.py) -, the slots
# shorten to that range (>= 4 mm before the wall) and the central one widens to pass the Ø22 pilot boss; the lid's
# window follows the motor (2 mm past the body toward the wrist, 2 mm past the connector).
DEFAULT = replace(
    LEGACY, roll=True,
    motor_x=LEGACY.web.wrist_x + centre_distance(RollEndParams().wrist_belt, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH),   # -136.37
    slide_range=(-141.5, -130.0),
    slot=SlotParams(centre_w=22.3, centre_x=(-148.0, -124.0), side_x=(-157.0, -114.5)),
    cap1=replace(LEGACY.cap1, window_len=53.0, window_offset=3.5, pocket_wrist_r=50.0),   # the body's wrist face clears the pocket wall
)
