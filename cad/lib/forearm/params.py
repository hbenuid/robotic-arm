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

from lib.bearings import (
    BEARING_6806_BORE,
    BEARING_6806_OD,
    BEARING_6806_SHOULDER_DIA,
    BEARING_6806_WIDTH,
    PULLEY_SEAT_SHIFT,
)
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_FACE_Y, GT2_PULLEY_90T_TEETH, centre_distance
from lib.cycloidal.params import MotorParams
from lib.fasteners import M3_CLEAR, M3_NUT, M3_SHCS, M4_CLEAR, M4_NUT, NutSize, ShcsSize
from lib.motors import MKS_SERVO42D_STACK, MKS_SERVO42D_W, MOTOR_40


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
    axes meet (42 − 17 along N from the elbow origin), so the three wrist axes stay concurrent. The rotor's pulley
    (parts/joints/forearm_roll_pulley: the roll's 90T ring, the output) puts its Ø39.7 spigot into a shallow locating
    recess on the wall's elbow face, the ring's face on the wall; 4x M3 from the wall's wrist face run through the wall
    and the spigot into nuts in the pulley's core (the bottom one's head in a channel under the web); the cables pass
    through the bore. The wall stands 48 mm
    along the roll axis from the elbow axis' station (and ForearmConfig.elbow_offset across from it): whatever the
    forearm's roll angle, it clears the upper arm's r 45 round end. It is a
    round flange about the roll axis on a foot as wide as itself - the web tapers to that width from the wrist boss
    (the neck) - braced by two gussets on the web's top face, either side of the wrist motor.
    The roll drive's frame, shaft and pulley: lib/forearm/roll.py."""

    axis_z: float = 25.0                  # [REFERENCE] the wrist centre's N-station above the elbow origin (42 − 17)
    wall_x: tuple = (-56.0, -48.0)        # [DESIGN] the wall's wrist face .. elbow face (8 thick): 48 from the elbow axis (>= 45 + 3: the upper arm's round end)
    wall_od: float = 60.0                 # [DESIGN] the wall's round outline about the roll axis: it covers the pulley's ring (its flanges Ø59.19)
    neck_half_w: float = 30.0             # [DESIGN] the web's half width at the wall (= wall_od / 2: the wall's foot); the web tapers to it from the wrist boss
    rib_y: tuple = (24.0, 30.0)           # [DESIGN] the two gussets on the web's top face against the wall's wrist face: |y| in this band - outside the
    #                                       wrist motor's body (+/- 21) and board (+/- 21.5), inside the neck
    rib_len: float = 30.0                 # [DESIGN] ... this far along the web from the wall
    rib_h: float = 20.0                   # [DESIGN] ... this high above the web at the wall (the wall's round outline cuts their tops)
    flange_dia: float = 39.7              # [DESIGN] the pulley's spigot on its front face, in the wall's recess
    flange_recess_add: float = 0.3        # [DESIGN] PETG mating clearance on the recess diameter (Ø40.0)
    flange_recess_depth: float = 2.0      # [DESIGN] a locating spigot, not a load path
    bolt_circle_dia: float = 31.0         # [DESIGN] 4x M3 on the axes (0 / 90 / 180 / 270 about the roll axis): through the pulley's spigot into
    #                                       its Ø44 core, the nuts in pockets from its bore (3 mm of the core outside them)
    bolt_count: int = 4
    bolt_angle_deg: float = 0.0
    bolt_dia: float = M3_CLEAR            # [DESIGN] 3.4, through the wall and the pulley's spigot to its nut
    screw: ShcsSize = M3_SHCS             # [DATASHEET] ISO 4762 M3, heads on the wall's wrist face (tools/bom.py EXTRAS)
    screw_len: float = 16.0               # [DESIGN] M3 x 16: through the wall (6 at the holes, less the recess) and the spigot into its nut in the
    #                                       pulley's core (RollDriveParams.end_nut), 2 pitches past it
    channel_clear: float = 0.5            # [DESIGN] the channel under the web for a screw whose head would land in the web (the bottom one): the head
    #                                       + this all round, open to the web's underside, from the wall into the motor slot (the screw lays in, a key reaches it)
    cable_bore: float = 24.0              # [DESIGN] round the rotor's Ø18 bore (RollDriveParams.bore)
    plug_clearance: float = 10.0          # [DESIGN] the wrist motor's connector plug needs this much room to the wall
    wrist_belt: int = 258                 # [ESTIMATE] 258-2GT closed belt, 6 mm: sets the motor slide (motor_x)


@dataclass(frozen=True)
class RollDriveParams:
    """The forearm roll drive (assemblies/forearm_roll_drive.py; docs/forearm_roll.md): ONE printed FRAME (stator, the
    forearm_roll_block) that is also the elbow's output flange, round a rotor split in two between its bearings (the
    elbow's 6806 pattern: two hubs meeting inside the lip). The roll motor sits ON the elbow axis, the roll axis the
    belt's centre distance above it (centre_distance = the arm's elbow offset, ForearmConfig.elbow_offset). The frame is
    one body round the motor: seen along the elbow axis round about it (end_r, the upper arm's round end) and tangent up
    to the tower round the roll axis, extruded from its underside (block_x[0], the elbow flange's - its underside repeats
    the SolidWorks j3_coupler's lip / boss / journal / stub into j1_link's bore, the elbow 90T bolts straight into it) to
    its open +N face; the motor in a pocket open on that face, the pocket's front wall its plate (the tension slots), the
    block's one flat front face the plate's; a round cup on that face round the roll axis, the 6806 pair in the tower
    under it. Rotor: the PULLEY (forearm_roll_pulley: the integral 90T ring sunk in the cup, its hub down through
    bearing 2, the spigot the forearm wall bolts onto - the roll's output) and the SHAFT (forearm_roll_shaft: a collar,
    its hub up through bearing 1 to the pulley's, its flange behind bearing 1 with the clamp's nuts and the stop lug);
    the rotor's own clamp - 4x M3 from the pulley's front face through both hubs into the shaft's nuts - holds the
    pulley, both inner rings and the shaft as one, the drive whole without the forearm; the forearm wall's 4x M3 into
    nuts in the pulley's core. MODULE FRAME: origin on the roll axis at the elbow axis' station
    (host (0, 0, axis_z)); +Z along the roll axis toward the wrist (host -X); +X = host +Z (N, away from the upper arm);
    +Y = host +Y = up in the arm's swing plane; the elbow axis runs along X through y = elbow_y (below the roll axis).
    Stations in that frame: layout.stack_positions(), the ring's set by the forearm wall (RollEndParams.wall_x), the
    rest by the ring. Assembly: docs/forearm_roll.md section 4."""

    # bearings: 2x 6806-2RS back to back on a lip in the tower (lib/bearings.py, the elbow's pair): bearing 2 right under
    # the ring, bearing 1 behind it; the rotor's two hubs meet in the lip's middle, spacing the inner rings as the lip
    # spaces the outer ones
    bearing_bore: float = BEARING_6806_BORE    # [DATASHEET] 6806-2RS (61806), 30 x 42 x 7
    bearing_od: float = BEARING_6806_OD
    bearing_width: float = BEARING_6806_WIDTH
    seat_add: float = 0.15                # [DESIGN] PETG press allowance on the seat diameter (cf. the drive's 6814 seat)
    journal_add: float = 0.3              # [DESIGN] the printed journals' interference in the inner rings (cf. the drive's hub)
    shoulder_dia: float = BEARING_6806_SHOULDER_DIA   # [DATASHEET] the rotor's shoulders bear on the inner rings only
    lip: float = 2.0                      # [DESIGN] between the pair's outer rings (the elbow's)
    lip_id: float = 37.6                  # [DESIGN] j1_link's elbow lip (37.64): clear of the hubs (Ø30.3) and the shoulders' Ø33
    run_gap: float = 1.0                  # [DESIGN] the ring's rear flange over bearing 2 and the cup's floor (the pulley's Ø33 shoulder spans it)
    # the frame: x along N (its underside at -X, 3.0 over the upper arm's flat top, host z -11), y up in the swing plane
    # (the motor below the roll axis, -Y), z along the roll axis
    block_x: tuple = (-33.0, 26.0)        # [DESIGN] host z -8 .. 51: the underside (the web's plane) .. the open +N face (the motor's
    #                                       connector, spun to +X, stands out of it)
    end_r: float = 45.0                   # [DESIGN] the round end about the elbow axis = j1_link's round end (concentric)
    tower_y: float = 29.0                 # [DESIGN] the tower's top over the roll axis: the bay + 3.4
    tower_corner_r: float = 6.0           # [DESIGN] its two corners in the profile
    tower_wall: float = 2.0               # [DESIGN] the tower's rear in the profile, behind the bay's floor
    web_t: float = 10.0                   # [DESIGN] the pocket's floor (the cradle's): 2 mm under the motor's -N side
    pocket_clear: float = 0.75            # [DESIGN] round the motor + board in the pocket, along Y past the tension travel
    pocket_rear: float = 1.0              # [DESIGN] behind the board's cover
    cup_od: float = 66.0                  # [DESIGN] the cup's boss on the front face round the roll axis (its -X side on the underside's plane)
    cup_id_add: float = 2.0               # [DESIGN] the cup round the ring's flange: + this on the diameter
    rim_under_teeth: float = 0.3          # [DESIGN] the cup's rim this far under the ring's teeth: the belt (its edge 0.5 above them) runs clear
    bay_r: float = 25.6                   # [DESIGN] the shaft's bay behind bearing 1, open through the +N face: round the stop lug's corners
    #                                       (r 24.6) + 1 (bearing 1 goes in from it)
    bay_clear: float = 1.0                # [DESIGN] the bay's floor behind the shaft's rear face by its hub's length through bearing 1 + this: the
    #                                       shaft, bearing 1 pressed on it, goes in through the +N face behind the tower and slides forward into the seat
    # the coupler features on the frame's underside (module -X) about the elbow axis (y = elbow_y), where j3_coupler#1 was
    # - measured on the SolidWorks coupler 2026-09-23 in j2_link's frame (host z = module x + axis_z); they turn in
    # j1_link's Ø80 recess / Ø42 bore
    lip_dia: float = 72.0                 # [DESIGN] a dust lip in j1_link's Ø80 recess (the coupler's Ø78 flange, kept inside the end's outline)
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
    nut_seat_x: float = -36.0             # [DESIGN] in the boss: the screw ends 2 pitches past its nut, under the motor's pocket
    nut_channel_past: float = 1.0         # [DESIGN] each channel runs up from its seat through the floor into the motor's pocket, this far past
    #                                       the floor: the nuts drop in from the pocket before the motor
    # the rotor: the pulley (the ring on its core, its hub through bearing 2, the spigot) and the shaft (a collar: its hub
    # through bearing 1, its flange behind it)
    bore: float = 18.0                    # [DESIGN] the cable bore through both: the clamp's M3 sit between it and the Ø30.3 journals
    ring_core_dia: float = 44.0           # [DESIGN] the pulley's core under the ring (the ring is an annulus fused on it)
    collar_od: float = 40.0               # [DESIGN] the shaft's flange behind bearing 1's Ø33 shoulder
    end_nut: NutSize = M3_NUT             # [DATASHEET] ISO 4032 M3: the nuts of the forearm wall's 4 screws (RollEndParams.screw), in the pulley's
    #                                       core (tools/bom.py EXTRAS) - each screw clamps the wall onto the ring's face
    end_nut_fit: float = 0.2              # [DESIGN] each nut's pocket - a slot from the cable bore out past the nut's corner, a flat either side
    #                                       (af wide) - its height and its reach past the corner; the nut bears on its wall-side face
    # the rotor's own clamp: 4x M3 from the pulley's front face (heads sunk in counterbores, under the wall's recess)
    # through both hubs into nuts in the shaft's flange - the pulley, both inner rings and the shaft as one
    clamp_circle_dia: float = 24.5        # [DESIGN] in the hubs, between the Ø18 bore and the Ø30.3 journals (1.55 / 1.2 mm of wall)
    clamp_angle_deg: float = 45.0         # [DESIGN] between the wall's screws (RollEndParams.bolt_angle_deg) and their nut pockets
    clamp_screw: ShcsSize = M3_SHCS       # [DATASHEET] ISO 4762 M3 (tools/bom.py EXTRAS)
    clamp_screw_len: float = 35.0         # [DESIGN] M3 x 35 from its counterbore's floor: the tip at the shaft's rear face (which it sets)
    clamp_head_clear: float = 0.25        # [DESIGN] the counterbores round the heads (on the radius) and over them
    clamp_nut: NutSize = M3_NUT           # [DATASHEET] ISO 4032 M3, in the shaft's flange: its pocket open to the rear face and the bore, the nut
    #                                       2 pitches short of the tip
    stop_lug_r: tuple = (19.5, 24.5)      # [DESIGN] the rotor's hard-stop lug on the shaft's flange (rooted 0.5 inside its Ø40), at +X in the zero pose
    stop_lug_t: float = 7.5               # [DESIGN] ... from the shaft's rear face
    stop_post_r: tuple = (21.6, 26.0)     # [DESIGN] the frame's post on the tower's rear face at -X, inside the bay (fused into its wall, r 25.5):
    #                                       they overlap r 21.6..24.5; clear of bearing 1 (r 21) and the shaft's flange (r 20)
    stop_post_t: float = 4.85             # [DESIGN] ... back from the tower's rear face: 2.5 of the lug's length beside it
    stop_deg_width: float = 10.0          # [DESIGN] angular width of each: contact at +/- (180 - width) = +/- stop_deg
    stop_deg: float = 170.0               # [ESTIMATE] = FOREARM_ROLL_LIMIT_DEG
    # the 90T ring (integral to the pulley; the 20T is flanged on its hub side only, so the ring carries two flanges)
    ring_teeth: int = 90
    ring_width: float = 7.0               # [DATASHEET] 6 mm belt
    ring_flange_dia: float = 59.19        # [REFERENCE] the SolidWorks 90T's flanges
    ring_flange_t: float = 1.2
    # the motor: ON the elbow axis, under the tower, centred on the roll axis in X, body toward the elbow (-Z), shaft
    # toward the wrist, spun motor_spin_deg about its axis so its cable connector points +X (out of the open face); the
    # pocket's front wall (the plate, normal to Z, in front of it) carries it - slotted along Y for belt tension
    motor: MotorParams = MOTOR_40
    motor_spin_deg: float = 90.0          # [DESIGN] about the motor axis: the connector (the motor frame's -Y) -> module +X
    board_w: float = MKS_SERVO42D_W       # [REFERENCE] 43: the MKS board's cover square (in the pocket with the motor)
    board_stack: float = MKS_SERVO42D_STACK   # [REFERENCE] 14.1: the board behind the motor's rear face
    roll_belt: int = 240                  # [ESTIMATE] 240-2GT closed belt, 6 mm: sets the centre distance (60.9) = the elbow axis' depth under the
    #                                       roll axis (ForearmConfig.elbow_offset - the arm's elbow offset)
    t20_hub: float = 10.95                # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    t20_len: float = 14.45                # [REFERENCE] vendor 20T: hub face .. its far end
    pulley_lift: float = 3.5              # [DESIGN] the 20T's hub face before the plate's front face: its far end at the motor shaft's tip, the tip
    #                                       behind the forearm wall's plane
    pad_t: float = 4.0                    # [DESIGN] the plate's thickness (the Ø22 x 2 pilot boss centres in it)
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
