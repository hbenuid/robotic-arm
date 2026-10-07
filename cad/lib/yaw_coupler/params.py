"""YawCouplerConfig - every dimension of the base_yaw coupler, j1_coupler (the yoke that turns on the base about the
base_yaw axis and holds the cycloidal drive - the shoulder_pitch stator: LEGACY cradles its housing, DEFAULT is a fork
round its turning shell), in its part frame.

Frame (= the SolidWorks part frame of j1_coupler, which placements.json places): origin on the base_yaw axis at the
height of the base's seat-ring top (lib/base/params.py CapParams.ring_top_y), +Y up the axis; the drive's axis (the
shoulder_pitch axis) runs along X at (y = YokeParams.axis_y, z = 0), its motor plate toward +X, its output hub
through the -X cheek.

The body, bottom up: the hub on the axis (a stub down into the upper base bearing, a bore, 4 small holes) under a
drafted disc (flats at x = +/-40, an ear of the Ø96 disc beyond each flat) with a recess in its underside; a ring on
the disc, then LEGACY's yoke flaring out of it - a -X cheek (outboard of the housing's output face) and a middle body
that holds the 8-pillar housing's bottom and +/-45 degree pillars in a channel and two V-grooves, all under the
housing's cradle, the nut pockets of the bolts through those pillars in the cheek's outer face, a pocket open to the
cradle over the hub - or DEFAULT's fork (ForkParams), the pocket over the hub kept.
The yoke's shapes follow the drive's housing (lib/cycloidal/params.py HousingParams: the cradle r against the bore,
the grooves and sockets on the pillars' sides, their points on the housing's od, the nut pockets on its bolt circle -
tests/yaw_coupler/ checks they agree).

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds: it stands on the base_yaw thrust bearing in the
base's groove (lib/bearings.py THRUST_*) - the recess's ceiling is the seat on the upper washer, wide enough to clear
the stack, and the rim is lifted clear of the base's top face (the SolidWorks rim sat on it, the coupler turning on
the base's face); its hub takes the base_yaw 120T as j3_coupler's the wrist's 90T: the stub the bearings' bore, on
through the lip to its lower face, where the 120T's hub end meets it (gt2_pulley_120t#1, lib/mounts.py), drilled for the
pulley's 4 bolts (M4 clearance, on its bolt circle at the diagonals, the nuts in hex pockets in the pocket's floor -
parts/base/yaw_pulley_screws / _nuts) round the pulley's bore; and its yoke is a fork round the drive whose shell
turns (lib/cycloidal/params.py DEFAULT_CONFIG, ShellParams): two legs past the shell's ends - the held hub bolts to
one, its disc the hub's own diameter; the other, its own part (parts/base/j1_motor_leg), a solid ring round the motor
plate's sleeve bolted to the base -, centred on the base_yaw axis with the drive's discs (ForkParams.face_x), the ring
lowered under the shell and run flat to flat - no cheek, no middle body, no cradle. The disc is turned with the legs:
drafted round to its rim (BASE_R, BASE_DRAFT; its flats and ears filled under the legs), its side carried on up each
leg's outer face.

Every number below was measured on the reference 2026-09-27 (face census; tests/yaw_coupler/test_j1_coupler.py
re-checks the builds against it): [REFERENCE] unless tagged. Three simplifications, all inside the reference-match
tolerance: the yoke's +/-Z outer faces are vertical and symmetric through the points on the housing's od (the
reference's lean 0.44 / 0.36 degrees), the flare is one cone over the whole yoke (the reference's last 3 mm toward +X
are the section at x 28.5 carried on), the channel has no 0.2 mm corner bevels. Units mm, degrees where named
*_deg. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.base.params import DEFAULT as _BASE
from lib.bearings import BEARING_6806_BORE, THRUST_OD, THRUST_STACK
from lib.belts import GT2_PULLEY_90T_BOLT_R
from lib.coupler.params import DEFAULT as _J3_COUPLER
from lib.fasteners import M4_CLEAR, M4_NUT


@dataclass(frozen=True)
class HubParams:
    """The underside and the hub on the axis: a recess in the disc's underside (inside the rim that stands on the
    base's top face), the stub down into the upper base bearing (a round on its end), the bore, 4 small holes on the
    diagonals - bore and holes from the stub's end up to the pocket's floor, the holes' nuts (if any) in hex pockets
    nut_depth down from that floor, a corner along Z."""

    recess_dia: float = 90.05
    recess_y1: float = 0.5           # the recess's ceiling (the rim below it: DiscParams.y0)
    stub_dia: float = 29.8
    stub_y0: float = -8.2            # the stub's end (up to the recess's ceiling)
    stub_round: float = 1.0          # r of the round on the end's outer edge
    bore_dia: float = 15.0
    hole_dia: float = 3.3
    hole_r: float = 10.600708        # 4 holes on this r ...
    hole_deg: float = 45.0           # ... the first at this angle (from +X toward +Z), 90 apart
    nut_af: float | None = None      # the holes' nut pockets (none in the SolidWorks part) ...
    nut_depth: float = 0.0           # ... this deep
    pulley_screw_len: float = 40.0   # [DESIGN] M4 x 40 (ISO 4762, parts/base/yaw_pulley_screws) from the floors of the base_yaw
    #                                  120T's counterbores (GT2_PULLEY_90T_HEAD_SEAT under its outer face): 33.7 ends flush with
    #                                  the nut's outer face (DEFAULT's pockets), 40 runs 6.3 past it into the pocket over the
    #                                  hub (an M4 x 35 ends 1.3 past: under two pitches)


@dataclass(frozen=True)
class DiscParams:
    """The disc: a band of band_r from the underside (y0) up to band_y1, then drafted in to top_r at top_y; cut by
    the flats x = +/- flat_x (the +X one on up through the ring). Beyond the flats the ears of an ear_r disc up to
    ear_y1. On the disc a ring of ring_r up to ring_y1, from x = ring_x0 (the cheek's outer face) to the +X flat."""

    y0: float = -0.4                 # the underside (the rim, flush on the base's top face)
    band_r: float = 53.082571
    band_y1: float = 0.0
    top_r: float = 50.843625
    top_y: float = 19.98843
    flat_x: float = 40.0
    ear_r: float = 48.0
    ear_y1: float = 8.0
    ring_r: float = 40.728657
    ring_y1: float = 24.82211
    ring_x0: float = -32.8


@dataclass(frozen=True)
class YokeParams:
    """The yoke on the ring, under the housing's cradle: a flare (a cone out of the ring's top edge, its apex on the
    axis at flare_apex_y) bounds its underside; the -X cheek (cheek_x) and the middle body (on to body_x1) are
    prisms along X between the outer faces z = +/- outer_z and the cradle (cradle_r about the drive's axis). Their
    tops, as points on circles about the drive's axis (given by |z|): the cheek from its outer face straight to
    cheek_top_z on the cradle - across the +/-45 degree pillars' far side; the middle body in a V-groove round each
    pillar - its end (a chord on the housing's od, groove_z[1] .. the outer face) and its near side (down to
    groove_z[0] on the cradle) - or, groove_z None, the cheek's top. The channel under the bottom pillar (its floor,
    and walls that open at the pillar's side slope) runs from the cheek to channel_x1; the notch round the bottom
    housing bolt's head from channel_x1 to the +X flat (neither unless channel). A closed socket round the drive's
    pillar at each of socket_deg (from straight down, toward +Z) - the pillar's sides and end socket_clear out
    (lib/yaw_coupler/layout.py socket_outline), open into the cradle - from the cheek through the middle body, in a
    wall socket_wall thick (under the socket's floor, near the +X end, it stands out of the flare: a rib). The
    pocket over the hub, open into the cradle. The nut pockets of the housing bolts at bolt_deg (from straight down,
    on bolt_circle_r) in the cheek's outer face, a hole on through the rest of it."""

    flare_apex_y: float = 10.285583
    cheek_x: tuple = (-32.8, -28.5)
    body_x1: float = 31.5
    outer_z: float = 53.131924
    axis_y: float = 90.0             # the drive's axis (along X)
    cradle_r: float = 58.0           # the housing's bore / 2 (the port's 116): the cradle is its circle
    od_r: float = 70.0               # the housing's od / 2 (the port's 140): the points of the V-grooves' ends
    cheek_top_z: float = 46.826264   # on the cradle
    groove_z: tuple | None = (34.224206, 45.574176)   # on the cradle, on od_r (None: no V-grooves)
    channel_floor_y: float = 20.204293
    channel_half_z: float = 5.552097           # at the floor ...
    channel_slope: float = 2.0 / 7.0           # ... opening dz / dy (the pillar's sides)
    channel_x1: float = 32.7
    channel: bool = True                       # the channel and the notch (the 8-pillar housing's bottom pillar)
    notch_r: float = 4.817811
    notch_y: float = 27.5
    pocket_half: tuple = (15.2, 31.0)          # x, z
    pocket_y0: float = 8.0
    bolt_circle_r: float = 62.5
    bolt_deg: tuple = (-45.0, 0.0, 45.0)       # from straight down (-Y), toward +Z
    nut_af: float = 7.196671
    nut_depth: float = 4.0
    bolt_hole_dia: float = 4.4
    socket_deg: tuple = ()                     # [DESIGN] a socket round the drive's pillar at each (none in SolidWorks) ...
    socket_clear: float = 0.2                  # [DESIGN] ... this clear of its sides and its end (the channel's 0.2) ...
    socket_wall: float = 2.0                   # [DESIGN] ... in a wall this thick, kept where the flare trims the yoke


@dataclass(frozen=True)
class ForkParams:
    """DEFAULT's yoke: a fork round the cycloidal drive whose shell turns (lib/cycloidal/params.py ShellParams), holding
    its two held ends on the drive's own axis with two LEGS, one past each end of the shell - each the drive's
    ShellParams.yoke_leg thick and end_plate_gap off its end, so the two mirror each other about the middle of the
    discs: a disc round the axis on a post down to the disc (leg_half_z either side). The disc is turned with the legs:
    its drafted side (DEFAULT's BASE_R, BASE_DRAFT) carries on up each leg's outer face, within the post, to
    flare_seat_flat under the hub's lowest bolt heads (lib/yaw_coupler/layout.py fork_flare_top), meeting the leg's
    flat face in a curved edge - the legs thick at the root, thinning to yoke_leg at the mounting area. The HUB leg (-X):
    its disc hub_plate_r, the output hub's own; the hub's 4x M4 x hub_screw_len (its arm-mount pattern) from
    counterbores in the outer face, their ends flush with the hub's captive nuts (ShellParams.hub_nut_depth). The MOTOR
    leg (+X) is its own part (parts/base/j1_motor_leg, lib/yaw_coupler/body.py build_motor_leg): a solid plate_r ring
    round the motor plate's sleeve (ring_bore_dia) - no split, no cap: it slides on over the motor and its board once the
    drive is in -, its post standing on the disc's top face (DiscParams.top_y) and its foot the disc's rim past the leg's
    outer face, butting the disc there (this part ends at that face, its ring cut back motor_leg_gap off the leg); 2x M4
    x motor_screw_len along the drive's axis at motor_bolt_y, motor_bolt_z either side of it, from counterbores in the
    foot into M4 nuts in slots from the disc's top face, under the leg's post. The drive sits with the middle of its discs
    on the base_yaw axis, so the two legs mirror each other about it too. Positions along the drive are the drive's z:
    this frame's x = face_x - z."""

    axis_y: float = 90.103183        # [REFERENCE] the drive's axis in this frame (placements.json cycloidal_drive#1), 0.21 off
    axis_z: float = 0.18573          # [REFERENCE] YokeParams' cradle axis (the 6-pillar housing sat in it; docs/open_issues.md)
    face_x: float = 24.0             # [DESIGN] the drive's z = 0 (the motor plate's outer face; its +Z is this frame's -X): the
    #                                  middle of the discs (the drive's z 24) on the base_yaw axis. The capture had it at
    #                                  CAPTURE_FACE_X: lib/placements.py SHIFTS moves the drive and all it carries the difference
    hub_plate_r: float = 35.15       # [DESIGN] the hub leg's disc: the output hub's od / 2 (lib/cycloidal/params.py
    #                                  OutputHubParams.od), flush with it - the hub's bolts on Ø50, their counterbores 6.45 inside its edge
    plate_r: float = 45.0            # [DESIGN] the motor leg's ring round the axis: 9.8 of wall round its bore
    leg_half_z: float = 25.0         # [DESIGN] each leg's post, either side of the axis
    flare_seat_flat: float = 1.5     # [DESIGN] the disc's draft up the legs stops this far under the hub's lowest bolt heads' seats
    ring_bore_dia: float = 70.4      # [DESIGN] the motor leg's ring round the sleeve: its 70 + 0.2 a side
    motor_leg_gap: float = 0.2       # [DESIGN] the disc's ring cut back this far off the motor leg's inner face
    motor_bolt_y: float = 10.0       # [DESIGN] the motor leg's 2 M4s along the drive's axis, this high ...
    motor_bolt_z: float = 12.0       # [DESIGN] ... this far either side of it (z) ...
    motor_head_seat: float = 4.5     # [DESIGN] ... their heads' seats this far out from the face the foot butts on (the clamped wall) ...
    motor_nut_wall: float = 1.6      # [DESIGN] ... into M4 nuts in slots from the disc's top face, this far in from that face ...
    motor_screw_len: float = 10.0    # [DESIGN] ... M4 x 10 (ISO 4762): through the foot and the wall, 0.7 past the nut
    hub_screw_len: float = 25.0      # [DESIGN] M4 x 25 (ISO 4762) from the hub leg's counterbores: flush with the hub's captive nuts


CAPTURE_FACE_X = 31.5   # [REFERENCE] where the SolidWorks capture put the drive's z = 0 in this frame (placements.json
#                         cycloidal_drive#1): the middle of its discs 7.5 off the base_yaw axis, toward the motor


@dataclass(frozen=True)
class YawCouplerConfig:
    hub: HubParams = HubParams()
    disc: DiscParams = DiscParams()
    yoke: YokeParams = YokeParams()
    fork: ForkParams | None = None   # DEFAULT: the fork round the turning-shell drive (the yoke's cheek, cradle and sockets go)


LEGACY = YawCouplerConfig()     # the SolidWorks part, exactly

# What the part builds: the coupler stands on the thrust bearing - washer, cage, washer (THRUST_STACK) on the floor of
# the base's groove - its seat (the recess's ceiling) on the upper washer, the recess THRUST_CLEAR a side round the
# stack's OD, the rim RIM_CLEAR over the base's top face. The part keeps its placement (placements.json puts its origin
# at the base's ring_top_y), so the seat is 1.1 higher than the SolidWorks ceiling, the stub 1.1 longer up into it.
THRUST_CLEAR = 0.2                # [DESIGN] the recess round the washers and the cage (radial)
RIM_CLEAR = 0.5                   # [DESIGN] the rim over the base's top face
RING_DROP = 2.0                   # [DESIGN] the ring's top lowered under the drive's turning shell: its pillars 2.68 over it
BASE_R = 58.0                     # [DESIGN] the disc at its rim (Ø116, the SolidWorks Ø106): room under the legs for their root ...
BASE_DRAFT = 12.0 / 63.0          # [DESIGN] ... and its side drafted in at dr/dy (10.8 degrees), carried on up the legs' outer
#                                   faces (x +/-46) to meet them at y 63 - the legs' root 20 thick, the motor leg's foot room for its M4s
_CAP = _BASE.cap                  # the base's frame: this part's origin at its ring_top_y
DEFAULT = replace(LEGACY,
                  hub=replace(LEGACY.hub, recess_dia=THRUST_OD + 2.0 * THRUST_CLEAR,
                              recess_y1=round(_CAP.groove_y0 - _CAP.ring_top_y + THRUST_STACK, 6),
                              # the hub the base_yaw 120T bolts to (the 90T's hub): the stub the bearings' bore, on to the lip's lower
                              # face (1.1 longer); the 90T's bolt circle at the diagonals (where the SolidWorks Ø3.3
                              # holes were - the nuts clear of the pocket's walls), M4 clearance, the nuts sunk flush
                              # in the pocket's floor in j3_coupler's press-fit pockets; the 90T's bore (the SolidWorks
                              # Ø15 left 1.3 of wall to the holes)
                              stub_dia=BEARING_6806_BORE, stub_y0=round(_BASE.bore.lip_y[0] - _CAP.ring_top_y, 6),
                              bore_dia=_J3_COUPLER.bore_dia, hole_dia=M4_CLEAR, hole_r=GT2_PULLEY_90T_BOLT_R,
                              nut_af=_J3_COUPLER.nut_af, nut_depth=M4_NUT.h),
                  # the ring RING_DROP lower: the drive's shell turns over it, its pillars sweeping 0.68 above the ring's top;
                  # flat to flat, symmetric under the centred fork (LEGACY's from the cheek's outer face); the disc
                  # BASE_R at its rim, drafted in at BASE_DRAFT (no band: the rim is lifted above band_y1) [DESIGN]
                  disc=replace(LEGACY.disc, y0=round(_CAP.top_y - _CAP.ring_top_y + RIM_CLEAR, 6),
                               ring_y1=round(LEGACY.disc.ring_y1 - RING_DROP, 6), ring_x0=-LEGACY.disc.flat_x,
                               band_r=BASE_R, top_r=round(BASE_R - BASE_DRAFT * (LEGACY.disc.top_y - LEGACY.disc.band_y1), 6)),
                  # the drive's shell turns (lib/cycloidal/params.py DEFAULT_CONFIG, ShellParams): the yoke holds its ends as
                  # a fork (ForkParams) - the cheek, the middle body, the cradle and the sockets round the 6-pillar
                  # housing's pillars go; YokeParams keeps the pocket over the hub
                  fork=ForkParams())
