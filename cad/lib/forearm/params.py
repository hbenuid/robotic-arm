"""ForearmConfig - every dimension of the forearm: j2_link (the web between the elbow and the wrist pivots) and
the forearm roll drive that splits it, in j2_link's part frame.

Frame (= the SolidWorks part frame of j2_link, which placements.json places): origin on the elbow pivot, the
wrist pivot at x = WebParams.wrist_x, +Z = N (the pitch-axis direction: the motor-body side; -Z = the belt side;
the upper arm lies at z < -8.5). With the roll joint (DEFAULT) the forearm sits elbow_offset across (+Y) from the
elbow: the elbow axis then runs along Z through y = -elbow_offset of this frame (lib/placements.py SHIFTS).

Two configurations: LEGACY reproduces the SolidWorks reference exactly (the part's REFERENCE_BUILD -
tests/test_reference_match.py), DEFAULT is what the part builds. In M1 they were the same; the forearm roll
(M2) makes DEFAULT end the forearm at a flange wall instead of the elbow disc, the link caps' removal
(2026-09-25: j2_cap_1 the lid, j2_cap_2 the belt tray) leaves out the sockets that located them, and SHORTENING
brings the wrist pivot nearer the elbow (lib/placements.py SHIFTS moves what lies beyond).

Every number below was measured on the references 2026-09-22 (planar / cylindrical face census;
tests/forearm/test_forearm_legacy.py re-checks the builds against them): [REFERENCE] unless tagged.
Units mm, degrees where named *_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import BEARING_6806_SHOULDER_DIA, PULLEY_SEAT_SHIFT
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_FACE_Y, GT2_PULLEY_90T_TEETH, centre_distance
from lib.cycloidal.params import MotorParams
from lib.fasteners import M3_CLEAR, M3_NUT, M3_SHCS, M4_CLEAR, M4_NUT, NutSize, ShcsSize
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
    the wall's elbow face; 4x M3 from the wall's wrist face run through the wall and the shaft's end into nuts buried
    in the shaft, behind bearing 2 (the bottom one's head in a channel under the web); the cables pass through the
    bore. The wall stands 48 mm
    along the roll axis from the elbow axis' station (and ForearmConfig.elbow_offset across from it): whatever the
    forearm's roll angle, it clears the upper arm's r 45 round end. It is a
    round flange about the roll axis on a foot as wide as itself - the web tapers to that width from the wrist boss
    (the neck) - braced by two gussets on the web's top face, either side of the wrist motor.
    Elbow block + shaft: lib/forearm/roll.py (M6)."""

    axis_z: float = 25.0                  # [REFERENCE] the wrist centre's N-station above the elbow origin (42 − 17)
    wall_x: tuple = (-56.0, -48.0)        # [DESIGN] the wall's wrist face .. elbow face (8 thick): 48 from the elbow axis (>= 45 + 3: the upper arm's round end)
    wall_od: float = 60.0                 # [DESIGN] the wall's round outline about the roll axis: it covers the stop lug and post in front of it (r 28 / 30)
    neck_half_w: float = 30.0             # [DESIGN] the web's half width at the wall (= wall_od / 2: the wall's foot); the web tapers to it from the wrist boss
    rib_y: tuple = (24.0, 30.0)           # [DESIGN] the two gussets on the web's top face against the wall's wrist face: |y| in this band - outside the
    #                                       wrist motor's body (+/- 21) and board (+/- 21.5), inside the neck
    rib_len: float = 30.0                 # [DESIGN] ... this far along the web from the wall
    rib_h: float = 20.0                   # [DESIGN] ... this high above the web at the wall (the wall's round outline cuts their tops)
    flange_dia: float = 39.7              # [DESIGN] the shaft's end spigot (< the 6808 bore: bearing 2 slides over it from the wrist end)
    flange_recess_add: float = 0.3        # [DESIGN] PETG mating clearance on the recess diameter (Ø40.0)
    flange_recess_depth: float = 2.0      # [DESIGN] a locating spigot, not a load path
    bolt_circle_dia: float = 31.0         # [DESIGN] 4x M3 on the axes (0 / 90 / 180 / 270 about the roll axis): each clearance hole centred in the
    #                                       shaft's neck wall, between its Ø24 bore and its Ø38 outside (1.8 mm either side)
    bolt_count: int = 4
    bolt_angle_deg: float = 0.0
    bolt_dia: float = M3_CLEAR            # [DESIGN] 3.4, through the wall and on through the shaft's end to its nut
    screw: ShcsSize = M3_SHCS             # [DATASHEET] ISO 4762 M3, heads on the wall's wrist face (tools/bom.py EXTRAS)
    screw_len: float = 25.0               # [DESIGN] M3 x 25: through the wall (6 at the holes, less the recess) and the shaft's end to its nut behind
    #                                       bearing 2 (RollDriveParams.end_nut), 2 pitches past it
    channel_clear: float = 0.5            # [DESIGN] the channel under the web for a screw whose head would land in the web (the bottom one): the head
    #                                       + this all round, open to the web's underside, from the wall into the motor slot (the screw lays in, a key reaches it)
    cable_bore: float = 24.0              # [DESIGN] = the shaft's bore
    plug_clearance: float = 10.0          # [DESIGN] the wrist motor's connector plug needs this much room to the wall
    wrist_belt: int = 258                 # [ESTIMATE] 258-2GT closed belt, 6 mm: sets the motor slide (motor_x)


@dataclass(frozen=True)
class RollDriveParams:
    """The forearm roll drive (assemblies/forearm_roll_drive.py; docs/forearm_roll.md): ONE printed FRAME (stator, the
    forearm_roll_block) that is also the elbow's output flange, round a hollow roll shaft (rotor). The roll motor sits ON
    the elbow axis and the roll housing above it, the belt's centre distance away (centre_distance = the arm's elbow
    offset, ForearmConfig.elbow_offset): the frame is the housing round the roll axis (bearing 1's seat in its rear end,
    the shaft's integral 90T ring in the Ø cavity_dia cavity OPEN to the front face, the belt window in its bottom
    wall), a web under the motor (on the upper arm's side, -X) down past the elbow axis whose underside repeats the
    SolidWorks j3_coupler's lip / boss / journal / stub into j1_link's bore - the elbow 90T bolts straight into it -, and
    the motor's plate in front of the motor (its tension slots); bearing 2 in the bolt-on END CAP on the housing's front
    face, the cables out of its rear end wall on the axis. MODULE FRAME: origin on the roll axis at the elbow axis'
    station (host (0, 0, axis_z)); +Z along the roll axis toward the wrist (host −X); +X = host +Z (N, away from the
    upper arm); +Y = host +Y = up in the arm's swing plane; the elbow axis runs along X through y = elbow_y (below the
    roll axis). Stations in that frame: layout.stack_positions(). Assembly: the elbow 90T's nuts into the web's channels
    from the motor's cradle, the motor into the cradle onto the plate, bearing 1 into the rear seat, the shaft in from
    the front (its rear journal into bearing 1, the ring through the cavity), bearing 2 over the spigot and the neck onto
    journal 2, the cap over it (4x M3 into the front face), the forearm wall onto the spigot (4x M3 into the nuts pushed
    into the shaft's pockets from its bore). Bearing 1 is pressed onto the shaft's rear journal first and rides in
    through the cavity and the core bore to its seat."""

    # bearings: 2x 6808-2RS - bearing 1 in the block's rear seat against its lip, bearing 2 in the cap's seat against the cap's lip
    bearing_bore: float = 40.0            # [DATASHEET] 6808-2RS (61808)
    bearing_od: float = 52.0
    bearing_width: float = 7.0
    seat_add: float = 0.15                # [DESIGN] PETG press allowance on the seat diameter (cf. the drive's 6814 seat)
    journal_add: float = 0.3              # [DESIGN] the printed journal's interference in the inner race (cf. the drive's hub)
    inner_race_od: float = 44.5           # [ESTIMATE] 6808 inner-race outer edge - the shaft's Ø44 core must not touch the outer race
    # the frame's housing: a rounded box round the roll axis - x along N (its underside at -X, the web's, rides 3.0 mm
    # above the upper arm's flat top, host z -11), y up in the swing plane (the motor below it, -Y), z along the roll axis
    block_x: tuple = (-33.0, 33.0)        # [DESIGN] host z -8 .. 58
    block_y: tuple = (-36.0, 36.0)        # [DESIGN] 5 mm of wall round the cavity; the motor's body clears the bottom at the slot's top end
    block_z: tuple = (-26.0, 36.0)        # [DESIGN] the rear end wall .. the front face (the cap sits on it): the bearings 57 apart; the housing's
    #                                       rear end is what the elbow motor meets first as the forearm lifts back (ELBOW_PITCH_LIMITS_DEG)
    block_corner_r: float = 8.0           # [DESIGN] the four edges along Z
    # the web under the motor: a slab on the upper arm's side (x block_x[0] .. + web_t) from the housing down past the
    # elbow axis, carrying the elbow flange (below) - the motor sits on it in a cradle between it, the plate and the housing
    web_t: float = 10.0                   # [DESIGN] 2 mm under the motor's -N side (its body +/- 21 about the roll axis in X)
    web_z: tuple = (-40.0, 36.0)          # [DESIGN] along the roll axis: over the lip about the elbow axis (lip_dia / 2 either side) .. the housing's front
    web_margin: float = 2.0               # [DESIGN] the web's lower end this far past the lip's edge
    # the coupler features on the web's underside (module -X) about the elbow axis (y = elbow_y), where j3_coupler#1 was -
    # measured on the SolidWorks coupler 2026-09-23 in j2_link's frame (host z = module x + axis_z); they turn in
    # j1_link's Ø80 recess / Ø42 bore
    lip_dia: float = 72.0                 # [DESIGN] a dust lip in j1_link's Ø80 recess (the coupler's Ø78 flange, kept inside the web's outline)
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
    pin_bore_x: tuple = (-47.0 - PULLEY_SEAT_SHIFT, -37.0)   # [DESIGN] from the stub's end, blind: stops 1 mm under the nut seats
    pulley_bolt_r: float = 11.0           # [REFERENCE] the elbow 90T's 4x M4 at r 11 (the SolidWorks coupler's pattern) ...
    pulley_bolt_deg: float = 45.0         # [DESIGN] ... turned off the axes, the elbow 90T with it (lib/mounts.py)
    pulley_bolt_dia: float = M4_CLEAR     # [DESIGN] 4.4, M4 clearance up through the stub, the journal and the boss to the nut seat
    pulley_hub_len: float = GT2_PULLEY_90T_FACE_Y[1] - GT2_PULLEY_90T_FACE_Y[0]   # [REFERENCE] 21.4, the 90T's length through its bolt holes:
    #                                       the screw heads sit in its counterbores (parts/joints/elbow_pulley_screws)
    pulley_screw_len: float = 35.0        # [DESIGN] M4 x 35 (ISO 4762) from the floors of the pulley's counterbores
    #                                       (GT2_PULLEY_90T_HEAD_SEAT under its outer face)
    nut_af: float = 6.85                  # [DESIGN] the M4 nuts' (ISO 4032, s 7) hex channels - j3_coupler's pocket (lib/coupler/params.py)
    nut_t: float = M4_NUT.h               # [DATASHEET] 3.2, ISO 4032 M4 nut height (parts/joints/elbow_pulley_nuts)
    nut_seat_x: float = -36.0             # [DESIGN] in the boss: the screw ends 2 pitches past its nut, under the motor's cradle
    nut_channel_past: float = 1.0         # [DESIGN] each channel runs up from its seat through the web into the motor's cradle, this far past
    #                                       the web's top: the nuts drop in from the cradle before the motor goes in
    # the housing bore, rear end wall -> front face
    end_wall: float = 3.0                 # [DESIGN] block_z[0] .. +3, the cable exit through it
    cable_exit_dia: float = 26.0          # [DESIGN] on the axis: the shaft's Ø24 bore + 1 mm all round
    lip: float = 2.0                      # [DESIGN] the lip bearing 1's outer race stops on, ID lip_id
    lip_id: float = 46.0
    core_bore_dia: float = 52.6           # [DESIGN] the bore from the seat to the cavity: bearing 1 (on the shaft) slides through it to its
    #                                       seat with 0.3 mm of radial clearance; the shaft's Ø44 core turns in it
    cavity_dia: float = 62.0              # [DESIGN] round the ring's flanges (+1.4), 2 mm of wall under it (block_x[0])
    cavity_z0: float = 16.0               # [DESIGN] the cavity's rear wall, 0.8 before the ring's first flange; open to the front
    belt_window_half_x: float = 22.0      # [DESIGN] the belt's two runs cross the bottom wall at |x| ~ 16..19 (tangent points on the ring at +/- 26.4, on the 20T at +/- 5.7)
    belt_window_y0: float = 29.0          # [DESIGN] from inside the cavity (y -29) out through the bottom wall
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
    end_nut: NutSize = M3_NUT             # [DATASHEET] ISO 4032 M3: the nuts of the forearm wall's 4 screws (RollEndParams.screw), in the Ø44 core
    #                                       behind bearing 2 (tools/bom.py EXTRAS) - the screws clamp the neck and journal 2 between head and nut
    end_nut_fit: float = 0.2              # [DESIGN] each nut's pocket - a slot from the cable bore outward, a flat either side (af wide): its height
    #                                       and its reach past the nut's outer corner over the nut; the nut bears on its wall-side face
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
    ring_z0: float = 18.0                 # [DESIGN] the teeth start here (sets the motor's station: its plate in front of it, t20 behind the ring)
    # the motor: ON the elbow axis, under the housing, centred on the roll axis in X, body toward the elbow (-Z), shaft
    # toward the wrist, spun motor_spin_deg about its axis so its cable connector points +X (away from the upper arm);
    # the frame's plate (normal to Z, in front of it) carries it - slotted along Y for belt tension
    motor: MotorParams = MOTOR_40
    motor_spin_deg: float = 90.0          # [DESIGN] about the motor axis: the connector (the motor frame's -Y) -> module +X
    roll_belt: int = 240                  # [ESTIMATE] 240-2GT closed belt, 6 mm: sets the centre distance (60.9) = the elbow axis' depth under the
    #                                       roll axis (ForearmConfig.elbow_offset - the arm's elbow offset)
    t20_hub: float = 10.95                # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    pulley_lift: float = 0.5              # [DESIGN] the 20T's hub face before the plate's front face
    pad_t: float = 4.0                    # [DESIGN] the plate's thickness (the Ø22 x 2 pilot boss centres in it)
    plate_w: float = 46.0                 # [DESIGN] the plate's width (X); it reaches plate_w / 2 (+ the slot's half) below the motor axis
    pad_slot_len: float = 5.0             # [DESIGN] +/- 2.5 belt-tension slide along Y
    pad_bolt_dia: float = 3.4
    pad_pilot_w: float = 22.3             # [DESIGN] the pilot boss slot

    @property
    def t20(self) -> float:
        """The 20T's tooth-band centre from the motor's mounting face: through the plate, the lift, the hub."""
        return self.pad_t + self.pulley_lift + self.t20_hub

    @property
    def centre_distance(self) -> float:
        """Ring axis .. motor axis: what the roll belt sets."""
        return centre_distance(self.roll_belt, self.ring_teeth, GT2_PULLEY_20T_TEETH)

    @property
    def elbow_y(self) -> float:
        """Module y of the elbow axis (along X): the whole centre distance below the roll axis - the motor sits on it."""
        return -self.centre_distance

    @property
    def motor_y(self) -> float:
        """The motor axis' module y: on the elbow axis (centred on the roll axis in X)."""
        return self.elbow_y


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
    elbow_offset: float = 0.0              # the roll axis' distance from the elbow axis along +Y (j2_link's, the module's): 0 = they cross
    #                                        (lib/placements.py SHIFTS moves the forearm and all beyond it across by DEFAULT - LEGACY)


LEGACY = ForearmConfig()     # the SolidWorks part, exactly

SHORTENING = 40.0   # [DESIGN] the wrist pivot this much nearer the elbow than the SolidWorks part's: the arm's reach
#                     is long for its NEMA 17 drives (the shoulder's and the elbow's holding torque)
_WEB = replace(LEGACY.web, wrist_x=LEGACY.web.wrist_x + SHORTENING)

# The forearm with the roll joint, SHORTENING shorter: the wall replaces the elbow disc; the wrist-pitch motor sits
# where the stock belt puts it (lib/belts.py) - between the wall, its connector plug clear by plug_clearance, and the
# wrist boss -, the slots shorten to the slide that leaves (>= 4 mm before the wall; its ends: the plug's clearance,
# 0.5 off the boss) and the central one widens to pass the Ø22 pilot boss; the caps are gone, so are the sockets
# that located them. The roll axis runs elbow_offset above the elbow axis - the roll motor sits on the elbow axis, the
# roll belt's centre distance under the ring - so the forearm and everything beyond it sit that far across (+Y).
DEFAULT = replace(
    LEGACY, roll=True, web=_WEB,
    motor_x=_WEB.wrist_x + centre_distance(RollEndParams().wrist_belt, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH),   # -99.52
    slide_range=(-103.5, -94.0),
    slot=SlotParams(centre_w=22.3, centre_x=(-110.0, -88.0), side_x=(-119.0, -78.5)),
    sockets=None,
    elbow_offset=RollDriveParams().centre_distance,   # [DESIGN] 60.9: the 240-2GT roll belt's centre distance
)
