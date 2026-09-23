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
    flange_dia: float = 60.0              # [DESIGN] the rotor flange (<= the block's Ø60 tube: the elbow end stays above j1_link)
    flange_recess_add: float = 0.3        # [DESIGN] PETG mating clearance on the recess diameter
    flange_recess_depth: float = 2.0      # [DESIGN] a locating spigot, not a load path
    bolt_circle_dia: float = 46.0         # [DESIGN] 4x M3 on the axes (0 / 90 / 180 / 270 about the roll axis)
    bolt_count: int = 4
    bolt_angle_deg: float = 0.0
    bolt_dia: float = 3.4                 # [DESIGN] M3 clearance (heads on the wrist face, nuts captive in the flange)
    cable_bore: float = 28.0              # [DESIGN] = the shaft's bore
    plug_clearance: float = 10.0          # [DESIGN] the wrist motor's connector plug needs this much room to the wall
    wrist_belt: int = 264                 # [ESTIMATE] 264-2GT closed belt, 6 mm: sets the motor slide (motor_x)


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
