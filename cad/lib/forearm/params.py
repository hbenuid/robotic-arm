"""ForearmConfig - every dimension of the forearm: j2_link (the web between the elbow and the wrist pivots) and
the forearm roll drive that splits it, in j2_link's part frame.

Frame (= the SolidWorks part frame of j2_link, which placements.json places): origin on the elbow pivot, the
wrist pivot at x = wrist_x (-210), +Z = N (the pitch-axis direction: the motor-body side; -Z = the belt side;
the upper arm lies at z < -8.5).

Two configurations: LEGACY reproduces the SolidWorks reference exactly (the part's REFERENCE_BUILD -
tests/test_reference_match.py), DEFAULT is what the part builds. In M1 they were the same; the forearm roll
(M2) makes DEFAULT end the forearm at a flange wall instead of the elbow disc, and the link caps' removal
(2026-09-25: j2_cap_1 the lid, j2_cap_2 the belt tray) leaves out the sockets that located them.

Every number below was measured on the references 2026-09-22 (planar / cylindrical face census;
tests/forearm/test_forearm_legacy.py re-checks the builds against them): [REFERENCE] unless tagged.
Units mm, degrees where named *_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import BEARING_6806_SHOULDER_DIA, PULLEY_SEAT_SHIFT
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, centre_distance
from lib.cycloidal.params import MotorParams
from lib.motors import MOTOR_40


@dataclass(frozen=True)
class WebParams:
    """The 11 mm web: a stadium between the two pivots (r = half_w at both ends)."""

    z0: float = 8.0            # bottom face (the belt side)
    z1: float = 19.0           # top face (the motor's mounting face - lib/params.py J2_MOTOR_WEB_FACE_Z)
    half_w: float = 45.0       # +/- y
    wrist_x: float = -210.0    # the wrist_pitch pivot (the elbow pivot is the origin)

    @property
    def thickness(self) -> float:
        return self.z1 - self.z0


@dataclass(frozen=True)
class ElbowDiscParams:
    """The elbow end (LEGACY): a disc under the web whose z=0 face bolts to j3_coupler#1 (4x M4 into captive nuts
    dropped in from the top). With the roll joint neither exists: the roll drive's block IS the elbow's output flange."""

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
    """Blind Ø5.18 x 2 locating sockets (dowel / magnet seats) on a grid the caps mirrored, plus a pair under
    the wrist boss. The web carries the grid on both faces. LEGACY only: the caps were removed 2026-09-25."""

    dia: float = 5.18
    depth: float = 2.0
    grid_x: tuple = (-30.0, -75.0, -120.0, -165.0)
    grid_y: float = 40.0
    wrist: tuple = ((-244.64, 20.0), (-244.64, -20.0))   # bottom face only (web + j2_cap_2)


@dataclass(frozen=True)
class RollEndParams:
    """The forearm roll's rotor side (roll=True): the forearm ends at a flange wall instead of the elbow disc.
    The roll axis runs along -X through (y 0, z axis_z) - the wrist centre, where the wrist_pitch and wrist_roll
    axes meet (42 − 17 along N from the elbow origin), so the three wrist axes stay concurrent. The rotor (the
    hollow roll shaft, parts/joints/forearm_roll_shaft) puts its Ø39.7 end spigot into a shallow locating recess on
    the wall's elbow face and takes 4x M3 through the wall; the cables pass through the bore. The wall stands 48 mm
    from the elbow axis - the least the rolling ±45 mm wall (corners r 57 about the roll axis) clears the upper arm's
    r 45 round end by 3 mm. Elbow block + shaft: lib/forearm/roll.py (M6)."""

    axis_z: float = 25.0                  # [REFERENCE] the wrist centre's N-station above the elbow origin (42 − 17)
    wall_x: tuple = (-56.0, -48.0)        # [DESIGN] the wall's wrist face .. elbow face (8 thick): 48 from the elbow axis (>= 45 + 3, see above)
    wall_z: tuple = (-10.0, 60.0)         # [DESIGN] full width y +/- half_w; sized under / over the former caps' tray and lid - nothing swings there
    flange_dia: float = 39.7              # [DESIGN] the shaft's end spigot (< the 6808 bore: bearing 2 slides over it from the wrist end)
    flange_recess_add: float = 0.3        # [DESIGN] PETG mating clearance on the recess diameter (Ø40.0)
    flange_recess_depth: float = 2.0      # [DESIGN] a locating spigot, not a load path
    bolt_circle_dia: float = 32.0         # [DESIGN] 4x M3 on the axes (0 / 90 / 180 / 270 about the roll axis), into the shaft's end wall
    bolt_count: int = 4
    bolt_angle_deg: float = 0.0
    bolt_dia: float = 3.4                 # [DESIGN] M3 clearance through the wall (heads on the wrist face; self-tapping in the shaft's end)
    cable_bore: float = 24.0              # [DESIGN] = the shaft's bore (the shaft's Ø39.7 end leaves ~8 mm of wall for the bolts)
    plug_clearance: float = 10.0          # [DESIGN] the wrist motor's connector plug needs this much room to the wall
    wrist_belt: int = 264                 # [ESTIMATE] 264-2GT closed belt, 6 mm: sets the motor slide (motor_x)


@dataclass(frozen=True)
class RollDriveParams:
    """The forearm roll drive (assemblies/forearm_roll_drive.py; docs/forearm_roll.md): ONE elbow block (stator) that
    is also the elbow's output flange - its underside repeats the SolidWorks j3_coupler's lip / boss / journal / stub
    down into j1_link's bore and the elbow 90T pulley bolts straight into it - round a hollow roll shaft (rotor) that
    CROSSES the elbow axis: bearing 1's seat in the block's rear end, the shaft's integral 90T ring in the Ø cavity_dia
    cavity that is OPEN to the front face (the ring passes through it on assembly), bearing 2 in the bolt-on END CAP on
    that face, the 40 mm kit motor on a vertical plate on the block's top - UP in the swing plane, its body behind the
    elbow axis -, the belt down through the top wall, the cables out of the rear end wall on the axis. MODULE FRAME:
    origin at the roll axis' crossing with the elbow axis (host (0, 0, axis_z)); +Z along the roll axis toward the
    wrist (host −X); +X = host +Z (N, away from the upper arm); +Y = host +Y = up in the arm's swing plane. Stations in
    that frame: layout.stack_positions(). Assembly: bearing 1 into the rear seat, the shaft in from the front (its
    rear journal into bearing 1, the ring through the cavity), bearing 2 over the spigot and the neck onto journal 2,
    the cap over it (4x M3 into the front face), the forearm wall onto the spigot (4x M3 into the shaft's end). Bearing 1
    is pressed onto the shaft's rear journal first and rides in through the cavity and the core bore to its seat."""

    # bearings: 2x 6808-2RS - bearing 1 in the block's rear seat against its lip, bearing 2 in the cap's seat against the cap's lip
    bearing_bore: float = 40.0            # [DATASHEET] 6808-2RS (61808)
    bearing_od: float = 52.0
    bearing_width: float = 7.0
    seat_add: float = 0.15                # [DESIGN] PETG press allowance on the seat diameter (cf. the drive's 6814 seat)
    journal_add: float = 0.3              # [DESIGN] the printed journal's interference in the inner race (cf. the drive's hub)
    inner_race_od: float = 44.5           # [ESTIMATE] 6808 inner-race outer edge - the shaft's Ø44 core must not touch the outer race
    # the block (stator): a rounded box round the roll axis - x along N (its underside at -X rides 0.5 mm above the
    # upper arm's slab, host z -8.5), y up in the swing plane (the motor plate stands on +Y), z along the roll axis
    block_x: tuple = (-33.0, 33.0)        # [DESIGN] host z -8 .. 58
    block_y: tuple = (-36.0, 36.0)        # [DESIGN] 5 mm of wall over the cavity; the motor's body clears the top at the slot's low end
    block_z: tuple = (-40.0, 36.0)        # [DESIGN] the rear end wall .. the front face (the cap sits on it)
    block_corner_r: float = 8.0           # [DESIGN] the four edges along Z
    # the coupler features on the block's underside (module -X), where j3_coupler#1 was - measured on the SolidWorks
    # coupler 2026-09-23 in j2_link's frame (host z = module x + axis_z); they turn in j1_link's Ø80 recess / Ø42 bore
    lip_dia: float = 72.0                 # [DESIGN] a dust lip in j1_link's Ø80 recess (the coupler's Ø78 flange, kept inside the block's outline)
    lip_x: tuple = (-35.0, -33.0)         # [REFERENCE] host -10 .. -8: 1.5 inside the recess (its floor at host -14.5), 2 proud of the underside
    boss_dia: float = 62.0                # [REFERENCE]
    boss_x: tuple = (-39.0, -35.0)        # [REFERENCE] host -14 .. -10
    journal_dia: float = 40.0             # [REFERENCE] in j1_link's Ø42 bore
    journal_x: tuple = (-40.3, -39.0)     # [REFERENCE] host -15.3 .. -14
    step_dia: float = BEARING_6806_SHOULDER_DIA   # [DESIGN] an inner-ring shoulder on the journal's face, down onto the upper elbow bearing ...
    step_x: tuple = (-41.0, -40.3)        # [DESIGN] ... host -16 .. -15.3: the journal's Ø40 face stopped 0.7 above that bearing (and bore on its outer ring)
    stub_dia: float = 30.0                # [REFERENCE] in the upper of the elbow's 6806-2RS pair (lib/mounts.py bearing_6806#3; the 90T's hub in #4)
    stub_x: tuple = (-47.0 - PULLEY_SEAT_SHIFT, -41.0)   # [REFERENCE] host -25 .. -16: the capture coupler's end (-47) on through the lip to the
    #                                       re-seated elbow 90T, which bolts flat onto it (its 2 mm in the lip space the inner rings)
    pin_bore_dia: float = 12.5            # [REFERENCE] the coupler's through bore
    pin_bore_x: tuple = (-47.0 - PULLEY_SEAT_SHIFT, -29.0)   # [DESIGN] from the stub's end, blind: stops 2.7 mm under the shaft's bore
    pulley_bolt_r: float = 11.0           # [REFERENCE] the elbow 90T's 4x M4 on the axes (host x / y)
    pulley_bolt_dia: float = 4.4          # [DESIGN] M4 clearance up through the stub, the journal and the boss
    insert_dia: float = 5.6               # [DESIGN] M4 heat-set insert hole
    insert_x: tuple = (-39.0, -31.0)      # [DESIGN] 8 deep from the boss's root; 2.2 mm from the cavity's rear wall
    # the housing bore, rear end wall -> front face
    end_wall: float = 3.0                 # [DESIGN] block_z[0] .. +3, the cable exit through it
    cable_exit_dia: float = 26.0          # [DESIGN] on the axis: the shaft's Ø24 bore + 1 mm all round
    lip: float = 2.0                      # [DESIGN] the lip bearing 1's outer race stops on, ID lip_id
    lip_id: float = 46.0
    core_bore_dia: float = 52.6           # [DESIGN] the bore from the seat to the cavity: bearing 1 (on the shaft) slides through it to its
    #                                       seat with 0.3 mm of radial clearance; the shaft's Ø44 core turns in it
    cavity_dia: float = 62.0              # [DESIGN] round the ring's flanges (+1.4), 2 mm of wall under it (block_x[0])
    cavity_z0: float = 16.0               # [DESIGN] the cavity's rear wall: 2.2 mm past the pulley-bolt inserts (z 8.2 .. 13.8); open to the front
    belt_window_half_x: float = 22.0      # [DESIGN] the belt's two runs cross the top wall at |x| ~ 16..19 (tangent points on the ring at +/- 26.4, on the 20T at +/- 5.7)
    belt_window_y0: float = 29.0          # [DESIGN] from inside the cavity out through the top wall
    belt_window_margin: float = 1.0       # [DESIGN] past the ring's flanges, both sides
    # the end cap (forearm_roll_retainer): the block's outline, seat 2 + lip, 4x M3 at the corners, the stop post on its outer face
    cap_lip: float = 2.0                  # [DESIGN] past bearing 2's seat, ID lip_id
    cap_bolt_inset: float = 6.0           # [DESIGN] the 4 bolts this far inside the outline's corners (inside the r 8 fillets)
    cap_bolt_dia: float = 3.4             # [DESIGN] through the cap
    cap_tap_dia: float = 2.5              # [DESIGN] M3 self-tapping in the block's front face
    cap_tap_depth: float = 8.0
    # the shaft (rotor)
    bore: float = 24.0                    # [DESIGN] the cable bore (= the wall's cable_bore)
    shoulder_od: float = 44.0             # [DESIGN] the core between the journals (< inner_race_od)
    neck_od: float = 38.0                 # [DESIGN] beyond journal 2 to the spigot: bearing 2 slides over it (< bearing_bore)
    shaft_end_clear: float = 1.0          # [DESIGN] the shaft's rear end .. the end wall's inner face (the cables exit there)
    end_bolt_tap_dia: float = 2.5         # [DESIGN] 4x M3 self-tapping into the shaft's end wall (RollEndParams.bolt_circle_dia)
    end_bolt_depth: float = 8.0
    stop_lug_r: tuple = (17.0, 28.0)      # [DESIGN] the rotor's hard-stop lug on the neck (rooted 2 mm inside its Ø38), at +X in the zero pose
    stop_post_r: tuple = (24.0, 30.0)     # [DESIGN] the cap's post on its outer face, at -X: they overlap r 24..28
    stop_t: float = 2.5                   # [DESIGN] both, from the cap's outer face; 0.5 mm short of the forearm wall
    stop_deg_width: float = 10.0          # [DESIGN] angular width of each: contact at +/- (180 - width) = +/- stop_deg
    stop_deg: float = 170.0               # [ESTIMATE] = FOREARM_ROLL_LIMIT_DEG
    # the 90T ring (integral; the 20T is flanged on its hub side only, so the ring carries two flanges)
    ring_teeth: int = 90
    ring_width: float = 7.0               # [DATASHEET] 6 mm belt
    ring_flange_dia: float = 59.19        # [REFERENCE] the SolidWorks 90T's flanges
    ring_flange_t: float = 1.2
    ring_z0: float = 18.0                 # [DESIGN] the teeth start here (the motor's body then ends 7.5 mm before the block's rear)
    # the motor: on the block's top, up in the swing plane, centred on the roll axis in X, body toward the elbow (-Z),
    # shaft toward the wrist, spun motor_spin_deg about its axis so its cable connector points +X (away from the upper
    # arm, clear of the block's top); a vertical plate (normal to Z) carries it - slotted along Y for belt tension -
    # and two cheeks flank the body (cheek_h stays under the connector)
    motor: MotorParams = MOTOR_40
    motor_spin_deg: float = 90.0          # [DESIGN] about the motor axis: the connector (the motor frame's -Y) -> module +X
    roll_belt: int = 240                  # [ESTIMATE] 240-2GT closed belt, 6 mm: sets the centre distance (60.9) = the motor's height
    t20_hub: float = 10.95                # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    pulley_lift: float = 0.5              # [DESIGN] the 20T's hub face above the plate's front face
    pad_t: float = 3.0                    # [DESIGN] the plate (on the shaft side of the mounting face; the Ø22 x 2 pilot boss centres in it)
    plate_w: float = 46.0                 # [DESIGN] the plate's width (X); it reaches plate_w / 2 above the motor axis
    pad_slot_len: float = 5.0             # [DESIGN] +/- 2.5 belt-tension slide along Y
    pad_bolt_dia: float = 3.4
    pad_pilot_w: float = 22.3             # [DESIGN] the pilot boss slot
    cheek_t: float = 3.0                  # [DESIGN] the two cheeks beside the motor body, standing on the block's top
    cheek_h: float = 16.0
    cheek_gap: float = 0.5                # [DESIGN] cheek .. motor body

    @property
    def t20(self) -> float:
        """The 20T's tooth-band centre from the motor's mounting face: through the plate, the lift, the hub."""
        return self.pad_t + self.pulley_lift + self.t20_hub

    @property
    def centre_distance(self) -> float:
        """Ring axis .. motor axis: what the roll belt sets."""
        return centre_distance(self.roll_belt, self.ring_teeth, GT2_PULLEY_20T_TEETH)

    @property
    def motor_y(self) -> float:
        """The motor axis' +Y offset - the whole centre distance (the motor is centred on the roll axis in X)."""
        return self.centre_distance


@dataclass(frozen=True)
class ForearmConfig:
    roll: bool = False                     # end the forearm at the roll flange wall instead of the elbow disc
    motor_x: float = -118.0                # the wrist-pitch motor's axis on the slide [ESTIMATE] (lib/params.py J2_MOTOR_SLIDE_X)
    slide_range: tuple = (-141.5, -62.5)   # motor-axis x the slots allow (bolt pattern inside the side slots)
    web: WebParams = WebParams()
    disc: ElbowDiscParams = ElbowDiscParams()
    boss: WristBossParams = WristBossParams()
    slot: SlotParams = SlotParams()
    sockets: SocketParams | None = SocketParams()   # None: no locating sockets
    roll_end: RollEndParams = RollEndParams()
    drive: RollDriveParams = RollDriveParams()


LEGACY = ForearmConfig()     # the SolidWorks part, exactly

# The forearm with the roll joint: the wall replaces the elbow disc; the wrist-pitch motor slides toward the wrist
# so its connector plug clears the wall - its position is what a stock 264-2GT belt sets (lib/belts.py) -, the slots
# shorten to that range (>= 4 mm before the wall) and the central one widens to pass the Ø22 pilot boss; the caps
# are gone, so are the sockets that located them.
DEFAULT = replace(
    LEGACY, roll=True,
    motor_x=LEGACY.web.wrist_x + centre_distance(RollEndParams().wrist_belt, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH),   # -136.37
    slide_range=(-141.5, -130.0),
    slot=SlotParams(centre_w=22.3, centre_x=(-148.0, -124.0), side_x=(-157.0, -114.5)),
    sockets=None,
)
