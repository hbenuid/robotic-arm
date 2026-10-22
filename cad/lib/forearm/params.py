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
from lib.belts import GT2_PULLEY_90T_FACE_Y
from lib.cycloidal.params import MotorParams, PETGTolerances
from lib.fasteners import M3_CLEAR, M3_NUT, M3_SHCS, M4_CLEAR, M4_NUT, M5_CLEAR, M5_NUT, M5_SHCS, NutSize, ShcsSize
from lib.motors import MKS_SERVO42D_STACK, MKS_SERVO42D_W, MOTOR_40, NEMA17_BOLT_SP


@dataclass(frozen=True)
class WebParams:
    """The 11 mm web: a stadium between the two pivots (r = half_w at both ends); with the roll joint, straight from the
    wall to its round wrist end."""

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
    Ø80 recess on top (the j3_coupler#2 side: the coupler's Ø78 flange dips into it, its dust lip turns in it)."""

    dia: float = 90.0
    z1: float = 33.5
    seat_dia: float = 42.2
    seat_z1: float = 27.5
    lip_dia: float = 37.65
    lip_z: tuple = (17.0, 19.0)
    recess_dia: float | None = 80.0   # None: no recess, the top face whole
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
    The roll axis runs along -X through (y 0, z axis_z) - the wrist centre on the wrist_pitch axis, where the
    SolidWorks wrist_roll axis met it (42 − 17 along N from the elbow origin; the wrist roll drive moved that axis off
    it: lib/wrist/params.py WristRollParams). The rotor's pulley
    (parts/joints/forearm_roll_pulley: the roll's 90T ring, the output) puts its Ø39.7 spigot into a shallow locating
    recess on the wall's elbow face, the ring's face on the wall; 4x M3 from the wall's wrist face run through the wall
    and the spigot into nuts in the pulley's core (the bottom one's head in a channel under the web); the cables pass
    through the bore. The wall stands 48 mm
    along the roll axis from the elbow axis' station (and ForearmConfig.elbow_offset across from it): whatever the
    forearm's roll angle, it clears the upper arm's r 45 round end. It is a
    round flange about the roll axis on a foot as wide as itself and the web (WebParams.half_w), round all the way
    down where the pulley's ring bears on it; braced by two gussets on the web's top face, either side of the wrist
    motor.
    The roll drive's frame, shaft and pulley: lib/forearm/roll.py."""

    axis_z: float = 25.0                  # [REFERENCE] the wrist centre's N-station above the elbow origin (42 − 17)
    wall_x: tuple = (-56.0, -48.0)        # [DESIGN] the wall's wrist face .. elbow face (8 thick): 48 from the elbow axis (>= 45 + 3: the upper arm's round end)
    wall_od: float = 60.0                 # [DESIGN] the wall's round outline about the roll axis: it covers the pulley's ring (its flanges Ø59.19) all
    #                                       round (no wider: the roll idler's screw head, 42.3 off the roll axis, reaches into the wall's band along it -
    #                                       a wider wall would meet it as the forearm rolls)
    rib_y: tuple = (24.0, 30.0)           # [DESIGN] the two gussets on the web's top face against the wall's wrist face: |y| in this band - outside the
    #                                       wrist motor's body (+/- 21) and board (+/- 21.5), inside the web
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
    plug_clearance: float = 10.0          # [DESIGN] the wrist motor's connector plug needs this much room beyond its connector (turned
    #                                       sideways, lib/mounts.py: clear of the web, the gussets and the wall)
    motor_wall_clear: float = 1.5         # [DESIGN] the wrist motor's body at least this far off the wall screws' heads (the screws are
    #                                       reached with the motor off)
    wrist_belt: int = 280                 # [DESIGN] 280-2GT closed belt, 6 mm (a multipack length, lib/belts.py STANDARD_2GT_LENGTHS): longer
    #                                       than the open loop at WRIST_MOTOR_CENTRES - the idler takes up the rest (IdlerParams)
    wrist_belt_z: float = 0.3             # [REFERENCE] the wrist belt's mid-plane: the wrist 90T's tooth band (its re-seated pose, lib/mounts.py
    #                                       gt2_pulley_90t#4), under the web - the motor's 20T level with it (tests/test_mounts.py)


@dataclass(frozen=True)
class IdlerParams:
    """[DESIGN] The wrist belt's tensioner (cad/AGENTS.md "Belts"): a smooth GT2 idler (parts/joints/gt2_idler_20t) on the
    belt's back, the stock wrist_belt being longer than the open loop at WRIST_MOTOR_CENTRES: the idler pushes the run
    leaving the motor's 20T into the loop until the belt fits (lib/belts.py idler_on_run, lib/forearm/layout.py
    wrist_idler_xy). It hangs under the web, level with the belt, on an M5 SHCS up from under it
    (parts/joints/wrist_idler_screw) through a rib under the web - the idler's top bears on it - and a slot through both,
    along y between slot_y's ends; into the nut (parts/joints/wrist_idler_nut) held flats-on in a channel down from the
    web's top face, under the motor's edge (the nut slides with the screw and cannot turn: the screw is turned from
    below)."""

    along: float = 24.0                   # its axis this far from the motor's along the belt's run toward the wrist: the nut's channel
    #                                       between the motor's pilot hole and the wrist boss
    slot_y: tuple = (4.7, 21.0)           # its axis' range along y, the slots' end centres: in 4 past the stock belt's place (a stretched
    #                                       belt), out 1 past just touching the open loop's strand (the belt goes on slack)
    rib_w: float = 7.0                    # [ESTIMATE] the rib's width: the idler's bearing inner ring only (the seller's drawing shows no bearing)
    slot_w: float = M5_CLEAR              # the axle's slot through the rib and the web
    screw: ShcsSize = M5_SHCS             # [DATASHEET] ISO 4762 M5 (parts/joints/wrist_idler_screw)
    screw_len: float = 20.0               # [DESIGN] M5 x 20: from its head under the idler through the nut, its tip under the motor's face
    nut: NutSize = M5_NUT                 # [DATASHEET] ISO 4032 M5 (parts/joints/wrist_idler_nut)
    nut_fit: float = 0.2                  # [DESIGN] the channel: the nut's flats + this
    nut_floor: float = 11.0               # [DESIGN] the channel's floor (the nut's bearing face): 3 of web under it


@dataclass(frozen=True)
class MotorHoleParams:
    """[DESIGN] A motor on round holes through a printed plate (its belt's idler tensions the belt, so its centre distance
    is fixed): the pilot boss's hole and the NEMA 17 square's four M3 clearance holes about the motor's axis, held by 4
    socket head cap screws through them from the plate's far face, the heads flush in counterbores - the elbow motor's
    screws and seat (lib/upper_arm/params.py ArmParams motor_screw, motor_pocket_dia). The wrist-pitch motor's through
    j2_link's web (ForearmConfig.holes, parts/joints/wrist_motor_screws: the web 11 thick, the elbow's), the roll motor's
    through the roll frame's plate (RollDriveParams.motor_holes, parts/joints/roll_motor_screws: pad_t)."""

    pilot_dia: float = 22.3               # the Ø22 pilot boss (SlotParams.centre_w)
    bolt_dia: float = M3_CLEAR            # 3.4
    bolt_half: float = NEMA17_BOLT_SP / 2.0   # 15.5: the 31 mm square
    screw: ShcsSize = M3_SHCS             # [DATASHEET] ISO 4762 M3, the heads in counterbores head_h deep: flush with the underside
    pocket_dia: float = MOTOR_40.motor_bolt_head_dia + PETGTolerances().bolt_clearance_add   # 5.7 [DESIGN] the counterbores:
    #                                       the drive's motor plate's head pockets (the elbow's)
    thread: float = MOTOR_40.bolt_hole_depth - MOTOR_40.motor_bolt_thread_margin   # 4.0 [DESIGN] the screws this far into the
    #                                       motor's tapped holes - the drive's rule (lib/params.py J1_MOTOR_SCREW_THREAD)


@dataclass(frozen=True)
class RollIdlerParams:
    """[DESIGN] The roll belt's tensioner (cad/AGENTS.md "Belts"): a smooth GT2 idler (parts/joints/gt2_idler_20t) on the
    belt's back on its return run (the ring back to the motor's 20T), level with the belt, pushed into the loop until the
    stock roll_belt fits the design's centre_distance (lib/belts.py idler_on_run; lib/forearm/layout.py roll_belt_loop,
    roll_idler_xy). It stands on a riser on the frame's front face - the riser's land under the idler's bearing inner
    ring - on an M5 SHCS down from its top (parts/joints/roll_idler_screw) through the idler, the riser's slot and the
    plate into a nut (parts/joints/roll_idler_nut) held flats-on in a channel from the frame's open +N face, under the
    face: the slot and the channel run along N (module X, the push), +/- travel to tension the belt, and the nut slides
    with the screw (turned from above)."""

    along: float = 31.7                   # its axis this far from the motor's along the belt's centre line, toward the ring: the
    #                                       nut's channel 5.3 off the motor's pocket, the idler 3.7 off the ring's flange
    travel: float = 3.0                   # +/- along module X (the push): the riser's slot and the nut's channel
    riser_w: float = 12.0                 # the riser's width across its slot (3.25 of wall either side of the M5)
    land_w: float = 7.0                   # [ESTIMATE] the land on the riser's top: the idler's bearing inner ring only (IdlerParams.rib_w)
    land_h: float = 1.0                   # the land over the riser: the idler's flange clear of the riser
    slot_w: float = M5_CLEAR              # the axle's slot through the land, the riser and the plate, on down past its tip
    screw: ShcsSize = M5_SHCS             # [DATASHEET] ISO 4762 M5 (parts/joints/roll_idler_screw), its head on the idler's top
    screw_len: float = 25.0               # [DESIGN] M5 x 25: through the idler, the riser, the roof and the nut, its tip 2 pitches+ past it
    nut: NutSize = M5_NUT                 # [DATASHEET] ISO 4032 M5 (parts/joints/roll_idler_nut)
    nut_fit: float = 0.2                  # [DESIGN] the channel: the nut's flats + this, its height + this
    roof: float = 3.0                     # [DESIGN] the plate over the channel: the nut's bearing face pulled up against it
    tip_clear: float = 1.0                # [DESIGN] the screw's hole on past its tip


ROLL_FRAME_R = 30.0   # [DESIGN] the roll frame's half width round the roll axis: its underside (6.0 over the upper arm's flat top),
#                       its open +N face and its tower's top - the frame symmetric about the roll axis, the bearing boss centred
#                       on its front face -, the forearm wall's radius (RollEndParams.wall_od / 2): the frame flush with the
#                       round parts that turn about the roll axis (RollDriveParams block_x, tower_y; tests/forearm/test_roll_drive.py test_stack)


@dataclass(frozen=True)
class RollDriveParams:
    """The forearm roll drive (assemblies/forearm_roll_drive.py; docs/forearm_roll.md): ONE printed FRAME (stator, the
    forearm_roll_block) that is also the elbow's output flange, round a rotor split in two between its bearings (the
    elbow's 6806 pattern: two hubs meeting inside the lip). The roll motor sits ON the elbow axis, the roll axis
    centre_distance above it (= the arm's elbow offset, ForearmConfig.elbow_offset): the stock roll belt round the motor's
    20T and the ring, an idler on its back on the frame's face taking up the rest (RollIdlerParams). The frame is
    one body round the motor: seen along the elbow axis an octagon about it (end_r, its flats on the upper arm's round end)
    hulled up to the tower round the roll axis, straight-sided, extruded from its underside (block_x[0], the elbow flange's - its underside repeats
    the SolidWorks j3_coupler's lip / boss / journal / stub into j1_link's bore, the elbow 108T bolts straight into it) to
    its open +N face; the motor in a pocket open on that face (a plug slot off each side at its board's side headers),
    the pocket's front wall its plate (round holes: its 4x M3 from the front face, the heads flush in counterbores -
    motor_holes, parts/joints/roll_motor_screws), the block's one flat front face the plate's - the 20T's teeth level
    with the ring (pulley_seat), the plate pad_t thick behind the face; a round bearing boss on that face round the roll axis, the 6806 pair in it and the tower under it. Rotor: the PULLEY (forearm_roll_pulley: the integral 90T ring over the boss, its hub down through
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
    run_gap: float = 1.0                  # [DESIGN] the ring's rear flange over bearing 2 and the bearing boss's top (the pulley's Ø33 shoulder spans it)
    # the frame: x along N (its underside at -X, 6.0 over the upper arm's flat top, host z -11), y up in the swing plane
    # (the motor below the roll axis, -Y), z along the roll axis
    block_x: tuple = (-ROLL_FRAME_R, ROLL_FRAME_R)   # [DESIGN] host z -5 .. 55: the underside (the web's plane) .. the open +N face,
    #                                       symmetric about the roll axis (the motor's connector, spun to +X, ends 2 inside the open pocket)
    end_r: float = 45.0                   # [DESIGN] the octagonal end about the elbow axis: its flats this far off it = j1_link's round end
    #                                       (tangent to it), its corners end_r / cos(22.5 deg) = 48.7 out (the elbow motor's board sweeps 56.5 out)
    end_sides: int = 8                    # [DESIGN] the end's polygon: an octagon, a flat on each axis (the front face's plane, the end, the bottom)
    tower_y: float = ROLL_FRAME_R         # [DESIGN] the tower's top over the roll axis, as far as the underside and the +N face: the bay + 4.4
    rear_line_r: float = 3.0              # [DESIGN] the rear line (the outline from the end up to the tower's top): tangent to the end_r
    #                                       circle and to a circle this radius in the tower's rear-top corner - where the round end's hull ran,
    #                                       2.3 off the elbow motor's board at the elbow's lower limit (lib/params.py ELBOW_PITCH_LIMITS_DEG);
    #                                       the octagon's corner behind it cut off, the tower's corners sharp (roll.py rear_line)
    tower_wall: float = 2.0               # [DESIGN] the tower's rear in the profile, behind the bay's floor
    web_t: float = 7.0                    # [DESIGN] the pocket's floor (the cradle's) over the underside: 2 mm under the motor's -N side
    pocket_clear: float = 0.75            # [DESIGN] round the motor + board in the pocket
    pocket_rear: float = 2.5              # [DESIGN] behind the board's cover: the motor + board go in from the +N face motor_drop low (the
    #                                       pilot boss under the plate), then lift into the plate's pilot hole
    motor_drop: float = 2.25              # [DESIGN] how low the motor slides in: its pilot boss (pilot_height) under the plate + 0.25
    shaft_slit: float = 6.0               # [DESIGN] the shaft's slit through the plate, from the pilot hole out through the +N face: the Ø5
    #                                       shaft passes it as the motor slides in (the screws' holes stay whole either side)
    bearing_boss_dia: float = 52.0        # [DESIGN] the boss on the front face round the roll axis, bearing 2's seat in it (4.9 of wall), its top
    #                                       bearing 2's front face: the ring's rear flange turns run_gap over it, 3.6 past its edge (no cup round
    #                                       the flange); 4 inside the frame's underside, +N face and tower top, on the flat of the front face
    bay_r: float = 25.6                   # [DESIGN] the shaft's bay behind bearing 1, open through the +N face: round the stop lug's corners
    #                                       (r 24.6) + 1 (bearing 1 goes in from it)
    bay_clear: float = 1.0                # [DESIGN] the bay's floor behind the shaft's rear face by its hub's length through bearing 1 + this: the
    #                                       shaft, bearing 1 pressed on it, goes in through the +N face behind the tower and slides forward into the seat
    # the coupler features on the frame's underside (module -X) about the elbow axis (y = elbow_y), where j3_coupler#1 was
    # - measured on the SolidWorks coupler 2026-09-23 in j2_link's frame (host z = module x + axis_z); they turn in
    # j1_link's Ø80 recess / Ø42 bore
    lip_dia: float = 72.0                 # [DESIGN] a dust lip over j1_link's Ø80 recess (the coupler's Ø78 flange, kept inside the end's outline)
    lip_x: tuple = (-35.0, -ROLL_FRAME_R)   # [REFERENCE] its foot at host -10, 1 over j1_link's flat top (the relief's floor, host -11), [DESIGN] up to
    #                                       the underside (host -5): 5 proud of it, bridging the gap over the upper arm's flat top
    boss_dia: float = 62.0                # [REFERENCE]
    boss_x: tuple = (-39.0, -35.0)        # [REFERENCE] host -14 .. -10
    journal_dia: float = 40.0             # [REFERENCE] in j1_link's Ø42 bore
    journal_x: tuple = (-40.3, -39.0)     # [REFERENCE] host -15.3 .. -14
    step_dia: float = BEARING_6806_SHOULDER_DIA   # [DESIGN] an inner-ring shoulder on the journal's face, down onto the upper elbow bearing ...
    step_x: tuple = (-41.0, -40.3)        # [DESIGN] ... host -16 .. -15.3: the journal's Ø40 face stopped 0.7 above that bearing (and bore on its outer ring)
    stub_dia: float = 30.0                # [REFERENCE] in the upper of the elbow's 6806-2RS pair (lib/mounts.py bearing_6806#3; the 108T's hub in #4)
    stub_x: tuple = (-47.0 - PULLEY_SEAT_SHIFT, -41.0)   # [REFERENCE] host -25 .. -16: the capture coupler's end (-47) on through the lip to the
    #                                       re-seated elbow 108T, which bolts flat onto it (its 2 mm in the lip space the inner rings)
    pin_bore_dia: float = 12.5            # [REFERENCE] the coupler's through bore
    pin_bore_x: tuple = (-47.0 - PULLEY_SEAT_SHIFT, -37.0)   # [DESIGN] from the stub's end, blind: stops 1 mm under the nut seats
    pulley_bolt_r: float = 11.0           # [REFERENCE] the elbow 108T's 4x M4 at r 11 (the SolidWorks coupler's pattern) ...
    pulley_bolt_deg: float = 45.0         # [DESIGN] ... turned off the axes, the elbow 108T with it (lib/mounts.py)
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
    stop_deg_width: float = 10.0          # [DESIGN] angular width of each at its outer radius - its sides parallel
    #                                       (lib/geom.py stop_wedge), so wider in: the post's inner corner meets the
    #                                       lug's side at +/- 168.3, not 180 - width (tests/test_sweeps.py holds it
    #                                       between the limit and 180 - width); FOREARM_ROLL_LIMIT_DEG stops 2 deg short
    #                                       of it
    # the 90T ring (integral to the pulley; the 20T is flanged on its hub side only, so the ring carries two flanges)
    ring_teeth: int = 90
    ring_width: float = 7.0               # [DATASHEET] 6 mm belt
    ring_flange_dia: float = 59.19        # [REFERENCE] the SolidWorks 90T's flanges
    ring_flange_t: float = 1.2
    # the motor: ON the elbow axis, under the tower, centred on the roll axis in X, body toward the elbow (-Z), shaft
    # toward the wrist, spun motor_spin_deg about its axis so its cable connector points +X (out of the open face); the
    # pocket's front wall (the plate, normal to Z, in front of it) carries it on round holes, its 4x M3 from the front face
    # (the idler tensions the belt: the motor does not slide)
    motor: MotorParams = MOTOR_40
    motor_holes: MotorHoleParams = MotorHoleParams()   # [DESIGN] the plate's pilot hole and the 31 mm square's 4 M3 holes, the screws'
    #                                       counterbores in its front face (the wrist-pitch motor's on j2_link's web)
    motor_spin_deg: float = 90.0          # [DESIGN] about the motor axis: the connector (the motor frame's -Y) -> module +X
    board_w: float = MKS_SERVO42D_W       # [REFERENCE] 43: the MKS board's cover square (in the pocket with the motor)
    board_stack: float = MKS_SERVO42D_STACK   # [REFERENCE] 14.1: the board behind the motor's rear face
    # the board's side plugs face the pocket's walls, along Y: a slot off each side at them (layout.plug_slots), open to
    # the +N face, so the motor + board go in with the cables plugged
    plug_clear: float = 0.5               # [DESIGN] the slots round each side header's outline
    plug_spare: float = 1.0               # [DESIGN] past its plug's room (lib/motors.py MKS_PLUG_ROOM)
    roll_belt: int = 280                  # [DESIGN] 280-2GT closed belt, 6 mm (a stock length, lib/belts.py STANDARD_2GT_LENGTHS): longer than the
    #                                       open loop at centre_distance - the idler takes up the rest
    centre_distance: float = 74.0         # [DESIGN] the ring's axis over the motor's (the elbow axis' depth under the roll axis = the arm's elbow
    #                                       offset, ForearmConfig.elbow_offset): the 280 belt with its idler (nearer than ~73 the idler meets the
    #                                       ring's flange and the belt's far run)
    idler: RollIdlerParams = RollIdlerParams()   # the roll belt's tensioner, on the frame's front face
    t20_hub: float = 10.95                # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    t20_len: float = 14.45                # [REFERENCE] vendor 20T: hub face .. its far end
    pulley_seat: float = 3.5              # [DESIGN] the 20T's hub face over the motor's pilot boss: the hub 1.5 down in the plate's pilot hole
    #                                       (pad_t - pilot_height - this), the motor pad_t behind the front face
    pad_t: float = 7.0                    # [DESIGN] the plate's thickness: 4 under the motor screws' heads + their counterbores (head_h), the
    #                                       heads flush in the front face - the plate grown 3 toward the motor, the face kept (the 20T's hub
    #                                       face 1.5 under it, its set screws above it); the screws: lib/params.py ROLL_MOTOR_SCREW_LEN

    @property
    def t20(self) -> float:
        """The 20T's tooth-band centre from the motor's mounting face: over the pilot boss, its seat, the hub."""
        return self.motor.pilot_height + self.pulley_seat + self.t20_hub

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
    motor_x: float = -118.0                # the wrist-pitch motor's axis [ESTIMATE] (lib/params.py J2_MOTOR_X)
    slide_range: tuple | None = (-141.5, -62.5)   # motor-axis x the slots allow (bolt pattern inside the side slots); None: round holes
    web: WebParams = WebParams()
    disc: ElbowDiscParams = ElbowDiscParams()
    boss: WristBossParams = WristBossParams()
    slot: SlotParams | None = SlotParams()   # the motor's slide through the web (None: round holes, MotorHoleParams)
    holes: MotorHoleParams | None = None     # the motor's round holes through the web
    sockets: SocketParams | None = SocketParams()   # None: no locating sockets
    roll_end: RollEndParams = RollEndParams()
    drive: RollDriveParams = RollDriveParams()
    idler: IdlerParams | None = None       # the wrist belt's idler under the web (None: no idler, its rib, slot and nut channel)
    elbow_offset: float = 0.0             # the roll axis' distance from the elbow axis along +Y (j2_link's, the module's): 0 = they cross
    #                                        (lib/placements.py SHIFTS moves the forearm and all beyond it across by DEFAULT - LEGACY)
    roll_turn_deg: float = 0.0             # the forearm and everything beyond it turned this far about the roll axis (j2_link's +X) from the
    #                                        capture (lib/placements.py TURNS); the roll drive keeps its pose


LEGACY = ForearmConfig()     # the SolidWorks part, exactly

SHORTENING = 45.0   # [DESIGN] the wrist pivot this much nearer the elbow than the SolidWorks part's: the arm's reach
#                     is long for its NEMA 17 drives (the shoulder's and the elbow's holding torque)
WRIST_MOTOR_CENTRES = 80.0   # [DESIGN] the wrist-pitch motor's axis from the wrist pivot: the stock wrist belt's own centre distance
#                              (81.97) less 1.97, so its idler has 3.75 of belt to take up (IdlerParams.slot_y); nearer the wrist
#                              its board comes closer to wrist_link's frame at the wrist pitch's lower limit (WRIST_PITCH_LIMITS_DEG)
_WEB = replace(LEGACY.web, wrist_x=LEGACY.web.wrist_x + SHORTENING,
               half_w=RollEndParams().wall_od / 2.0)   # [DESIGN] straight from the wall, as wide as it, no taper
_BOSS = replace(LEGACY.boss, dia=2.0 * _WEB.half_w,       # [DESIGN] as wide as the web: its round wrist end
                z1=LEGACY.boss.seat_z1,                   # [DESIGN] the bearing seat's top: j3_coupler#2's dust lip turns 0.5 over it,
                recess_dia=None)                          #          its Ø78 flange 4.5 - no recess for either

# The forearm with the roll joint, SHORTENING shorter: the wall replaces the elbow disc; the web runs straight from
# the wall to the wrist, the boss as wide as it; the wrist-pitch motor sits WRIST_MOTOR_CENTRES from the wrist on round
# holes, its connector turned sideways (lib/mounts.py), the stock belt round an idler under the web (IdlerParams); the
# caps are gone, so are the sockets that located them. The roll axis runs
# elbow_offset above the elbow axis - the roll motor sits on the elbow axis, the drive's centre distance under the
# ring (the stock belt round its tensioner) - so the forearm and everything beyond it sit that far across (+Y). The forearm is turned half a turn about the
# roll axis from the capture: the wrist-pitch motor on the upper arm's side of the roll axis, the wrist belt on the
# other, the wrist and the gripper rolled with it; the wall's 4 bolts at 0 / 90 / 180 / 270 deg take the roll pulley as
# before.
DEFAULT = replace(
    LEGACY, roll=True, web=_WEB, boss=_BOSS,
    motor_x=_WEB.wrist_x + WRIST_MOTOR_CENTRES,   # -85
    slide_range=None, slot=None, holes=MotorHoleParams(), idler=IdlerParams(),
    sockets=None,
    elbow_offset=RollDriveParams().centre_distance,   # [DESIGN] 74: the roll drive's centre distance (the 280 belt with its idler)
    roll_turn_deg=180.0,                              # [DESIGN] the wrist-pitch motor on the other side of the forearm
)
