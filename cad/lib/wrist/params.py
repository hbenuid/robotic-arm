"""WristConfig - every dimension of the wrist body, wrist_link (the wrist_pitch stage: its plate carries the wrist_pitch
output flange on the pitch axis, j3_coupler#2 retired - LEGACY: the coupler's flange bolted to it), in its part frame, of the wrist roll drive it carries (WristRollParams) and of the
gripper's connector to it (ConnectorParams, gripper_j3_connector's own frame).

Frame (= the SolidWorks part frame of wrist_link, which placements.json places): origin on the plate's underside at
the centre of the tower's round end, +Z up through the plate toward the coupler and the forearm; the wrist_pitch axis
is the line (PlateParams.pitch_x, 0) along Z, the SolidWorks wrist_roll axis the line (y 0, z EndFaceParams.axis_z)
along X, toward +X (the pancake motor, the gripper). Symmetric about y = 0 but for the back opening (TowerParams.back_y).

LEGACY (the SolidWorks part): a plate with a round end on each axis - j3_coupler's seat on the pitch axis (a through
hole clear of the wrist 90T's nuts, the flange's 4 M4 into hex nut pockets in the underside); on the other end the
tower: an R39 wall round a bore, a wedge under a slope inside it (the slope's edge with the bore filleted), cheeks
either side of a slot in the bore, and a block out to the end face gripper_clamp_bracket bolted to (it held the pancake
on the roll axis) - the NEMA 17 pattern's M3 and the bracket's M4, drilled through the whole part along X (the
SolidWorks through-all: the z 6.5 pair score the plate's top).

Two configurations: LEGACY reproduces the SolidWorks reference (the part's REFERENCE_BUILD -
tests/test_reference_match.py); DEFAULT is what the part builds: the wrist roll drive's frame - the pancake ON the
pitch axis, the roll axis R' beside it (WristRollParams; docs/wrist_roll.md) - its wall on the forearm's side the plate
(only where the frame runs: no plate stands past it), the wrist_pitch output flange on that wall's coupler face
(FlangeParams: the post, the shoulder, the stub - the elbow's pattern), the rest of the frame on its underside (the free
face, away from the forearm); the tower, its end face and the bracket are gone (lib/placements.py SHIFTS moves the
gripper onto R').

Every number below was measured on the reference 2026-09-27 (face / vertex census; tests/wrist/test_wrist_link.py
re-checks the builds against it): [REFERENCE] unless tagged. SolidWorks' float noise (<= 6e-4) is rounded off the end
face's holes and the block. Units mm. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import BEARING_6806_BORE, BEARING_6806_OD, BEARING_6806_SHOULDER_DIA, BEARING_6806_WIDTH
from lib.belts import GT2_PULLEY_20T_TEETH, centre_distance
from lib.coupler.params import DEFAULT as COUPLER
from lib.fasteners import M3_CLEAR, M3_NUT, M3_SHCS, M4_CLEAR, NutSize, ShcsSize
from lib.motors import MKS_SERVO42D_STACK, MKS_SERVO42D_W
from lib.style import GRILL_PITCH, GRILL_SLOT


@dataclass(frozen=True)
class PlateParams:
    """The plate, z 0..thickness: a round end on the pitch axis, flats at y = +/-half_width, a round end on the origin
    (the tower's wall). SolidWorks drew the pitch end 0.0007 over the flats' half width: the flats clip it."""

    thickness: float = 5.0
    pitch_x: float = -45.445774        # the wrist_pitch axis (x, 0), j3_coupler#2's
    pitch_r: float = 39.000699         # the round end on it (DEFAULT: the roll frame's octagonal end about it, its flats this far
    #                                    off the axis - lib/wrist/roll.py side_profile)
    half_width: float = 39.0


@dataclass(frozen=True)
class SeatParams:
    """j3_coupler's seat on the pitch axis: a through hole (clear of the wrist 90T's nuts in the coupler flange's
    underside), the flange's 4 M4 on the axes at bolt_r - holes from the plate's top down to hex nut pockets
    nut_depth deep in the underside, a flat toward the axis."""

    hole_dia: float = 29.885736
    bolt_r: float = 35.0
    bolt_dia: float = 4.1
    nut_af: float = 6.85
    nut_depth: float = 2.0


_T = PlateParams.thickness   # the plate's coupler face, where FlangeParams' stations start
# [DESIGN] wrist_link and everything it carries (the wrist roll drive, the gripper) this far along its +Z (toward the
# forearm) from the capture, on the wrist pitch axis (WristConfig.pitch_inset, lib/placements.py SHIFTS): its coupler face
# FACE_GAP (2) off j2_link's wrist boss (trimmed flush with the upper wrist bearing: lib/forearm/params.py BOSS_TRIM) - the
# capture's 14.5, the SolidWorks coupler's flange and lip ring between them, + the trim's 1.5 - so the flange's post is
# that much shorter; with the elbow's (lib/forearm/params.py ELBOW_INSET) the gripper's roll axis in the plane of the
# base_yaw axis at home. The 6806 stack in the boss, the wrist 90T and its screws stay with the forearm.
PITCH_INSET = 14.0


@dataclass(frozen=True)
class FlangeParams:
    """[DESIGN] The wrist_pitch output flange (DEFAULT; j3_coupler#2 retired, as the elbow's j3_coupler#1 is) - its own
    printed part, wrist_flange, on the roll frame's coupler face about the pitch axis, part z along it: the post (the
    coupler's journal carried down to the frame), its foot chamfered, the shoulder on the upper wrist bearing's inner ring,
    the stub on through j2_link's lip to the wrist 90T, which bolts onto its end - its 4 M4 through the flange into nuts
    in the frame's plate, in hex channels open down into the pancake's pocket (the nuts drop in from there, before the
    pancake): the 90T, the flange and the frame one clamp. Its own part so that its round faces print as circles
    (printed on the stub's end): wrist_link prints its bearing seats upright, and keeps nothing round about the pitch
    axis but the M4 clearance holes (the elbow's alike: lib/forearm/params.py RollDriveParams post_*). The post stands on
    the frame alone, cut flat where it passes the frame's front face and rear (nothing past the frame's outline seen along
    the pitch axis); under it the frame's FOOT (lib/wrist/link.py _foot) carries the post's front flat on down into the
    front facet's foot - an octagon with its corners on the post's Ø40, no arc of it; a small octagonal KEY on the coupler
    face inside the bolt circle (key_*) in a recess in the flange's foot centres it and keys it against turning. The
    90T's bolt pattern is turned pulley_bolt_deg off the axes, so the front nut keeps its wall to the front flat. The
    stations are the retired coupler's (lib/coupler/params.py DEFAULT) on the coupler face, less PITCH_INSET (the wrist
    that much nearer the forearm): the 6806 stack, the 90T and its screws' heads stay; the post is 1.3 long, the stub 9
    - the elbow flange's design (lib/forearm/params.py RollDriveParams post_x / stub_x). The pancake 1.25 under the 5 mm
    plate leaves no room for the elbow's spigot and M4 x 35: the nuts sit in the plate, the screws M4 x 33."""

    post_dia: float = COUPLER.journal_dia                              # [REFERENCE] Ø40 over j2_link's Ø42.2 seat
    post_z: tuple = (_T, _T + COUPLER.journal_y1 - PITCH_INSET)        # the coupler face .. 0.7 over the upper wrist bearing (the
    #                                                                    frame's foot under it from the free face, lib/wrist/link.py _foot)
    post_chamfer: float = 1.0                                          # [DESIGN] its foot's 45 deg chamfer: EDGE_CHAMFER (lib/style.py)
    #                                                                    would leave the 1.3 post no straight band
    step_dia: float = COUPLER.step[0]                                  # the inner-ring shoulder ...
    step_z: tuple = (_T + COUPLER.journal_y1 - PITCH_INSET, _T + COUPLER.step[1] - PITCH_INSET)   # ... down onto the upper wrist
    #                                                                    bearing's top face
    stub_dia: float = COUPLER.stub_dia                                 # through the upper wrist bearing and j2_link's lip ...
    stub_z: tuple = (_T + COUPLER.step[1] - PITCH_INSET, _T + COUPLER.stub_y1 - PITCH_INSET)   # ... to the 90T's face (the lip's
    #                                                                    lower face): 9 long
    pulley_screw_len: float = 33.0                                     # [DESIGN] the 90T's 4 M4 x 33 (parts/joints/wrist_pulley_screws)
    #                                                                    from the floors of its counterbores, through the flange into
    #                                                                    the plate's nuts: the tips 2 pitches past them, 0.65 over the
    #                                                                    pancake. Not an ISO 4762 length ...
    pulley_screw_bought: float = 35.0                                  # [DESIGN] ... so bought as M4 x 35, cut to pulley_screw_len
    pin_bore_dia: float = COUPLER.bore_dia                             # [REFERENCE] blind from the stub's end ...
    pin_bore_z1: float = _T + COUPLER.step[1] - PITCH_INSET + 1.0      # ... to 1 over the stub's base (where it stopped when the
    #                                                                    nuts sat there)
    pulley_bolt_r: float = COUPLER.pulley_bolt_r                       # [REFERENCE] the 90T's 4 M4 at r 11 ...
    pulley_bolt_deg: float = 45.0                                      # [DESIGN] ... turned off the axes, the wrist 90T, its screws
    #                                                                    and nuts with it (lib/mounts.py; the elbow's RollDriveParams
    #                                                                    pulley_bolt_deg): on the axes the front nut's corner came 0.55
    #                                                                    off the post's front flat, turned it keeps 3.9
    pulley_bolt_dia: float = M4_CLEAR                                  # 4.4, as the elbow's (the coupler's 4.1 was the SolidWorks pattern)
    nut_af: float = COUPLER.nut_af                                     # the channels: a press for the M4 nut (s 7) ...
    nut_corner_deg: float = 0.0                                        # ... a corner this far off the pattern's turn (pulley_bolt_deg
    #                                                                    off +X): the coupler's pockets, as parts/joints/wrist_pulley_nuts
    #                                                                    models the nuts, turned with the 90T
    nut_seat_z: float = _T - 1.0                                       # [DESIGN] the nuts' faces toward the 90T, in the plate: 1 of it
    #                                                                    between each nut and the flange - the screws' tips 2 pitches
    #                                                                    past: each channel from here down
    channel_past: float = 1.0                                          # ... through the plate, this far into the pancake's pocket
    key_af: float = 15.0                                               # [DESIGN] the key on the coupler face: an octagon about the pitch
    #                                                                    axis (a flat on each axis), this across flats - inside the bolt
    #                                                                    circle (1.3 off the M4 holes at its flats), under the pin bore
    key_h: float = 1.0                                                 # [DESIGN] ... this tall, in the flange's recess key_h deep
    key_fit: float = 0.15                                              # [DESIGN] the recess over the key, a side


@dataclass(frozen=True)
class TowerParams:
    """The tower on the origin end: the wall (r wall_r) from its back face at x0 on, z1 high, round a bore (r bore_r)
    open above a slope - the plane from (slope_x0, the plate's top) up to (slope_x1, z1), the bore's edge on it filleted
    fillet_r (a wedge of material under it); cheeks inside the bore either side of a slot (y = +/-slot_half), their
    front faces at cheek_x (the back of the bore behind them opened down to the slope - on +Y out to back_y, on -Y to
    the bore: SolidWorks' one asymmetry), up to cheek_z1; the block the bracket bolts to, out to its end face at
    block_x1, +/-block_half_width wide, block_z high."""

    x0: float = 0.948349
    z1: float = 44.0
    wall_r: float = 39.0
    bore_r: float = 30.0
    slope_x0: float = 2.407088
    slope_x1: float = 30.0             # at z1 the slope touches the bore (bore_r)
    fillet_r: float = 10.0
    slot_half: float = 21.0
    cheek_x: float = 5.0
    cheek_z1: float = 43.0
    back_y: float = 30.0
    block_x1: float = 40.0             # the end face: gripper_clamp_bracket's seat
    block_half_width: float = 32.0
    block_z: tuple = (1.0, 43.0)


@dataclass(frozen=True)
class EndFaceParams:
    """The end face's holes, on the roll axis (y 0, z axis_z): the NEMA 17 pattern's M3 on an m3_sp square (the
    bracket's motor) - the rows at z = axis_z + r * m3_sp / 2 for r in m3_rows -, the bracket's M4 at y = +/-m4_y,
    z = axis_z +/- m4_dz; every one drilled along X through the whole part."""

    axis_z: float = 22.0
    m3_sp: float = 31.0
    m3_dia: float = 3.2
    m3_rows: tuple = (-1.0, 1.0)
    m4_y: float = 26.5
    m4_dz: float = 15.5
    m4_dia: float = 4.2


@dataclass(frozen=True)
class PancakeParams:
    """The wrist-roll pancake (parts/wrist/nema17_pancake: vendor/nema17_pancake.step) as the frame needs it, in the
    vendor file's own frame - measured on it 2026-10-14 [REFERENCE]: the axis along +Z through (0, axis_y), the rear
    plate's face at rear_z, the mounting face at face_z (the Ø22 pilot proud of it by pilot_height), the four tapped M3
    on the bolt_sp square, the cable connector's boss plug_d proud of the body square's -Y side."""

    axis_y: float = 12.6594
    rear_z: float = 12.6594
    face_z: float = 35.6594
    body_w: float = 41.5
    pilot_dia: float = 22.0
    pilot_height: float = 1.0
    bolt_sp: float = 31.0               # = lib/motors.py NEMA17_BOLT_SP
    plug_d: float = 5.5
    plug_w: float = 16.2                # [REFERENCE] the connector's boss across (along the body's side), measured on the vendor STEP
    plug_h: float = 8.5                 # [REFERENCE] ... and from the rear plate's face toward the mounting face, 2026-11-04

    @property
    def body_length(self) -> float:
        """Its rear face to its mounting face (23)."""
        return self.face_z - self.rear_z


@dataclass(frozen=True)
class WristBackPanelParams:
    """[DESIGN] The wrist frame's removable back panel (parts/wrist/wrist_back_panel), the forearm roll frame's
    (lib/forearm/params.py BackPanelParams) one joint further out: a plate flush with the frame's slanted rear over the
    tower - from the tower's far side to the board's window, stopping at the plate's free face - in a step `t` deep,
    over the shaft's bay bored through the rear: lift it off and the shaft, its clamp nuts and the stop are open to reach,
    and the shaft goes in or out along R'. Located by a ring rim on its inner face inside the bay's bore, held by magnet
    pairs (parts/wrist/wrist_back_magnets): one disc in a pocket in the step's floor, one in a pocket in the panel's inner
    face, each pressed to its pocket's bottom. A notch at its edge toward the motor lets the gripper servo's cable out of
    the rotor's bore (it joins the motor's cables there and crosses the wrist joint near its axis); a second over the MKS
    board's near side plug's room. A grill of straight slots (lib/style.py GRILL_SLOT / GRILL_PITCH) is cut through it
    over the bay, inside the rim, the turning shaft behind it. Positions are module (x, y), on the rear plane
    (lib/wrist/roll.py rear_plane)."""

    t: float = 2.5                      # the panel's thickness = the step's depth, square to the rear plane
    side_clear: float = 0.1             # off the plate's free face
    gap: float = 0.2                    # its edge short of the board's window
    rim_clear: float = 0.3              # the locating rim inside the bay's bore ...
    rim_w: float = 1.6                  # ... this wide (4 perimeters) ...
    rim_h: float = 3.0                  # ... this tall from the panel's inner face (the shaft's flange 10+ in front of it)
    magnet_dia: float = 3.0             # [DATASHEET] N52 NdFeB disc magnet, 3 x 2 mm (the forearm roll panel's)
    magnet_t: float = 2.0
    pocket_dia: float = 3.2             # each magnet's pocket (glued in, pressed to its bottom) ...
    pocket_depth: float = 2.1           # ... in the step's floor and in the panel's inner face: 0.4 of the panel outside it
    magnet_points: tuple = ((-22.0, -18.0), (22.0, -18.0), (-20.0, 19.0), (22.0, 19.0))   # (x, y) round the bay's bore (3.4+ of
    #                                     wall to it and to the coupler face), the -X pair on the motor's side clear of the plate
    notch_w: float = 6.0                # the gripper servo's cable's notch, from its edge toward the motor ...
    notch_reach: float = 9.0            # ... this far inside the bay's bore; a groove under it across the step's floor ...
    notch_groove: float = 2.0           # ... this deep, into the board's window
    grill_clear: float = 1.6            # the grill's field (lib/wrist/roll.py back_grill): the octagon about R' with its corners this
    #                                     far inside the ring rim's inner edge, its edge toward the motor this far short of the cable's notch
    grill_w: float = GRILL_SLOT         # the grill's slots (lib/style.py), along y (toward the motor), through along R' ...
    grill_pitch: float = GRILL_PITCH    # ... at this pitch along x, one on R'


@dataclass(frozen=True)
class WristRollParams:
    """The wrist roll drive (assemblies/wrist_roll_drive.py; docs/wrist_roll.md), the forearm roll drive's pattern one
    joint further out: the pancake ON the wrist_pitch axis, on the plate's free face (the side away from the forearm),
    belted to the roll axis beside it - centre_distance across in the wrist's swing plane, at the motor's height - so the
    wrist roll axis R' no longer meets the pitch axis (a wrist offset: the wrist is no longer spherical). wrist_link IS
    the frame (the stator), laid out as the forearm roll's frame (lib/forearm/roll.py build_block): one body - seen along
    the pitch axis an octagon about it (pitch_r) hulled to the tower round R', its wall on the forearm's side the plate
    (the wrist_pitch output flange on it: FlangeParams; no plate past the frame), from the plate's coupler face to the
    closed far (+X) face (side_t), its rear one slanted plane (rear_low,
    rear_bend), the front face's long edges faceted (front_facet). The pancake's pocket opens down through the BOTTOM -
    the frame's end beyond the motor, away from R' - and the pancake slides in from there, its shaft up a slit along the
    tension slots; its front wall the plate the pancake bolts to (slotted along Y: the belt's tension), a groove inside
    the side wall for its plug; no wall behind it: a window there - open at the rear and the bottom - takes the pancake's
    MKS board, on its rear face. The tower round R': the 6806 pair back to back on a lip, a bearing boss on the front
    face, the shaft's bay behind bearing 1 bored on through the frame's rear (the shaft goes in from behind), closed by a
    removable BACK PANEL flush in a step on the rear (back: WristBackPanelParams - its notch lets the gripper servo's
    cable out of the rotor's bore), the hard stop's post in the bay. The rotor: the
    PULLEY (wrist_roll_pulley: its hub through bearing 2, the 40T band, the face gripper_j3_connector bolts to - the
    roll's output) and the SHAFT (wrist_roll_shaft: its hub through bearing 1, its flange in the bay with the rotor
    clamp's nuts and the stop lug); the clamp's 4x M3 from the pulley's face through both hubs hold them and both inner
    rings as one.

    MODULE FRAME: origin on R' at the pitch axis' station; +Z along R' toward the gripper (wrist_link's +X); +X away from
    the plate (the far face, wrist_link's -Z); +Y = wrist_link's +Y; R' lies `side` x centre_distance along it from the
    pitch axis, so the pitch axis runs along X at y = -side x centre_distance - the motor sits on it, centred on R' in X.
    Stations: lib/wrist/layout.py stack_positions()."""

    side: float = -1.0                  # [DESIGN] which side of the pitch axis R' (the gripper) sits on, along wrist_link's Y: -1
    #                                     the outer side - away from the upper arm when the elbow folds, the forearm turned -,
    #                                     where the wrist pitch's long range bends the gripper down (+1, the first build, put
    #                                     it on the inner side); the frame builds as the other side's mirror image, the rotor
    #                                     and every bought row placed, not mirrored
    belt: int = 158                     # [DESIGN] 158-2GT closed belt, 6 mm (a stock length, lib/belts.py STANDARD_2GT_LENGTHS): its
    #                                     centre distance at the motor slots' middle (no idler: docs/open_issues.md)
    ring_teeth: int = 40                # [DESIGN] the pulley's printed 40T band: 2 : 1 from the pancake's 20T (WRIST_ROLL_RATIO)
    ring_width: float = 7.0             # [DATASHEET] 6 mm belt
    ring_flange_dia: float = 29.0       # [DESIGN] the band's two flanges (the 20T is flanged on its hub side only)
    ring_flange_t: float = 1.2
    ring_core_dia: float = 23.0         # [DESIGN] the core under the band (its root ~Ø23.5): the band is an annulus fused on it
    axis_h: float = 27.0                # [DESIGN] R' (and the motor's axis) over the plate's coupler face (wrist_link z 5 - axis_h): the bay's
    #                                     floor 4.4 over that face, the motor 1.25 over the free face
    # bearings: 2x 6806-2RS back to back on a lip in the tower (lib/bearings.py, the belt joints' and the forearm
    # roll's): bearing 2 right under the band, bearing 1 behind it; the rotor's two hubs meet in the lip's middle
    bearing_bore: float = BEARING_6806_BORE   # [DATASHEET] 6806-2RS (61806), 30 x 42 x 7
    bearing_od: float = BEARING_6806_OD
    bearing_width: float = BEARING_6806_WIDTH
    shoulder_dia: float = BEARING_6806_SHOULDER_DIA   # [DATASHEET] the rotor's shoulders bear on the inner rings only
    seat_add: float = 0.15              # [DESIGN] PETG press allowance on the seat diameter (the forearm roll's)
    journal_add: float = 0.3            # [DESIGN] the printed journals' interference in the inner rings (the forearm roll's)
    lip: float = 2.0                    # [DESIGN] between the pair's outer rings
    lip_id: float = 37.6                # [DESIGN] clear of the hubs (Ø30.3) and the shoulders' Ø33
    run_gap: float = 1.0                # [DESIGN] the rotor off the frame (the pulley's Ø33 shoulder spans it over bearing 2)
    # the motor: centred on the pitch axis (its body's middle at module z 0), its plug spun to +X (toward the side wall)
    pancake: PancakeParams = PancakeParams()
    motor_spin_deg: float = 90.0        # [DESIGN] about its axis: the plug (the vendor frame's -Y) -> module +X, into a groove inside
    #                                     the side wall (plug_clear) - turned to the bottom, the board's side plugs would face the plate
    pocket_clear: float = 0.75          # [DESIGN] round the body in the pocket, along Y past the tension travel
    pocket_rear: float = 1.0            # [DESIGN] behind its rear face (past motor_drop)
    motor_drop: float = 1.25            # [DESIGN] how far back the pancake slides in from the bottom: its pilot behind the plate + 0.25
    plug_clear: float = 0.5             # [DESIGN] the plug's groove round its boss (plug_d proud of the body), past the tension travel
    #                                     along Y, from the plug down out through the bottom and the rear (its lead to the board)
    # its MKS SERVO42D on its rear face (the board's encoder reads the shaft's rear end), turned with it (motor_spin_deg:
    # its motor plug toward the pancake's, toward the side wall; the side plugs along Y), in a window through the frame's
    # rear - the board's housing square + the tension travel + board_clear, open at the rear and the bottom like the pocket:
    # they go in together
    board_w: float = MKS_SERVO42D_W     # [REFERENCE] 43: the board's housing square, from the pancake's rear face back
    board_stack: float = MKS_SERVO42D_STACK   # [REFERENCE] 14.1: the board behind the pancake's rear face
    board_clear: float = 0.45           # [DESIGN] round the board's housing in the window, along Y past the tension travel: the
    #                                     window's wall to the shaft's bay keeps 1.43 (pocket_clear's 0.75 would leave 1.13)
    shaft_slit: float = 6.0             # [DESIGN] the Ø5 shaft's slit through the plate, from the pilot slot down out through the bottom
    pad_t: float = 4.0                  # [DESIGN] the plate (the pocket's front wall) the pancake's 4x M3 bolt through
    pad_slot_len: float = 5.0           # [DESIGN] +/- 2.5 along Y: the belt's tension (no idler)
    pad_bolt_dia: float = 3.4
    pad_pilot_w: float = 22.3           # [DESIGN] the pilot's slot
    pulley_seat: float = 0.5            # [DESIGN] the 20T's hub face over the pilot (the forearm roll's)
    t20_hub: float = 10.95              # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    # the frame round the motor and the tower (lib/wrist/roll.py side_profile, tower_section): seen along the pitch axis the
    # plate's octagon about it hulled to the tower round R', from the plate's coupler face to the closed far (+X) face;
    # seen along R' the tower the octagon about it, the motor's side bounded by the plate and the far face
    frame_r: float = 30.0               # [DESIGN] the tower's octagon about R' (lib/style.py SIDES, these flats): the far face (x frame_r),
    #                                     the tower's far side
    side_t: float = 7.5                 # [DESIGN] the far face's wall over the pocket and the board's window: its inside 22.5 off R', outside
    #                                     the pancake's body + pocket_clear (21.5) and the board's housing + board_clear (21.95)
    front_facet: float = 7.0            # [DESIGN] the front face's two long edges faceted this far at 45 deg (the forearm roll frame's): 4+ of
    #                                     wall left to the pocket's corner; the bearing boss carried down through them (it rises out of them)
    rear_low: float = -18.5             # [DESIGN] the frame's rear behind the motor (module z): the plate's octagon cut there - as deep as
    #                                     the wrist pitch's +38 deg limit allows (2.4 off the forearm's wrist board there, tests/test_sweeps.py)
    rear_bend: float = 25.0             # [DESIGN] the rear one slanted plane from the tower's far rear corner (z_rear) down to rear_low
    #                                     this far from R' toward the motor - past the back panel's edge (the panel lies flat), flat beyond
    bearing_boss_dia: float = 52.0      # [DESIGN] round R' on the front face, bearing 2's seat in it (the forearm roll's)
    bay_r: float = 22.7                 # [DESIGN] the shaft's bay behind bearing 1, bored on through the frame's rear: the stop lug's corners
    #                                     (r 21.68: its flat end's) + run_gap all round; bearing 1 (r 21) passes through it with the shaft, in
    #                                     from behind (the back panel off)
    bay_clear: float = 1.0              # [DESIGN] with tower_wall, the frame's rear at the tower's far side (z_rear): this + tower_wall
    tower_wall: float = 2.0             # [DESIGN] behind the shaft's hub length behind its rear face (the station the wrist pitch's +38 deg
    #                                     limit was set against: tests/test_sweeps.py)
    back: WristBackPanelParams = WristBackPanelParams()   # the back panel over the bay
    # the rotor
    bore: float = 10.0                  # [DESIGN] the cable bore through both pieces (the gripper servo's 3-pin plug passes): the clamp's
    #                                     M3 between it and the band's core
    collar_od: float = 36.0             # [DESIGN] the shaft's flange behind bearing 1's Ø33 shoulder
    clamp_circle_dia: float = 17.0      # [DESIGN] the rotor clamp's 4x M3 inside the band's core (1.3 of core outside them, 1.8 inside)
    clamp_angle_deg: float = 45.0       # [DESIGN] between the connector's bolts
    clamp_screw_len: float = 35.0       # [DESIGN] M3 x 35 (ISO 4762) from its counterbore's floor: its tip sets the shaft's rear face
    clamp_head_clear: float = 0.25      # [DESIGN] round the heads and over them
    nut_fit: float = 0.2                # [DESIGN] each M3 nut's pocket: a slot (af wide) past its corner and over its height by this
    stop_lug_r: tuple = (17.5, 21.6)    # [DESIGN] the rotor's hard-stop lug on the shaft's flange (rooted 0.5 inside it), at +X in the zero pose
    stop_lug_t: float = 4.5             # [DESIGN] ... from the shaft's rear face: 1.35 short of bearing 1's outer ring
    stop_post_r: tuple = (19.5, 23.1)   # [DESIGN] the frame's post at -X in the bay, fused into its wall; clear of the flange (r 18)
    stop_post_t: float = 3.85           # [DESIGN] ... back from bearing 1's seat: 2.5 of the lug's length beside it
    stop_deg_width: float = 10.0        # [DESIGN] each at its outer radius (sides parallel, as the forearm roll's)
    # the face gripper_j3_connector's hub bolts to: 4x M3 (connector_bolt_*), their nuts in radial pockets from its rim
    face_dia: float = 42.0              # [DESIGN]
    face_t: float = 6.0                 # [DESIGN] in front of the band's front flange
    face_nut_depth: float = 1.0         # [DESIGN] each nut's bearing face this far under the face
    connector_turn_deg: float = 176.818   # [REFERENCE] the connector's plate x axis in the module's x-y plane (the gripper keeps its
    #                                     capture orientation: tests/wrist/test_roll_drive.py checks it against the placements)
    clamp_screw: ShcsSize = M3_SHCS     # [DATASHEET] ISO 4762 M3 (tools/bom.py EXTRAS)
    nut: NutSize = M3_NUT               # [DATASHEET] ISO 4032 M3: the clamp's in the shaft's flange, the connector's in the face
    bolt_dia: float = M3_CLEAR          # [DESIGN] 3.4: the clamp's and the connector's M3 through the rotor

    @property
    def centre_distance(self) -> float:
        """R' from the motor's axis: the stock belt's centre distance (lib/belts.py), the motor at its slots' middle."""
        return centre_distance(self.belt, self.ring_teeth, GT2_PULLEY_20T_TEETH)

    @property
    def t20(self) -> float:
        """The 20T's tooth-band centre from the pancake's mounting face: over the pilot, its seat, the hub."""
        return self.pancake.pilot_height + self.pulley_seat + self.t20_hub


@dataclass(frozen=True)
class ConnectorParams:
    """gripper_j3_connector (the gripper module's link to the wrist roll's output), in its SolidWorks part frame - measured
    on reference/solidworks/gripper_j3_connector.step 2026-10-14 [REFERENCE]: a plate y 0..plate_t (a Ø disc_dia disc cut
    to +/- half_width in x), the servo holder's and the cover's holes through it along Y; LEGACY a Ø boss_dia boss on its
    +Y face (y plate_t..boss_y1) whose 20-tooth bore slid onto the pancake's 20T (and 1 mm into the plate) - modelled as
    the circle of its area (bore_r_eq: the teeth are not); DEFAULT (hub set) a hub instead, bolted to the wrist roll
    pulley's face: a core, then a flange with the 4x M3 (heads on its plate side, reached past the plate's flats), the
    gripper servo's cable through its bore."""

    disc_dia: float = 46.0
    half_width: float = 11.5
    plate_t: float = 4.5
    holes: tuple = ((-2.5, 18.5, 3.0166), (-2.5, -18.5, 3.2), (4.5, 18.5, 3.2), (4.5, -18.5, 3.2))   # (x, z, Ø)
    recess_depth: float = 1.0           # LEGACY: the bore's recess into the plate's +Y face
    boss_dia: float = 14.0              # LEGACY
    boss_y1: float = 12.5               # LEGACY
    bore_r_eq: float = 5.7418           # LEGACY: sqrt(103.574 / pi) - the 20-tooth bore's area (the recess floor's)
    hub: bool = False                   # DEFAULT: the bolted hub
    hub_core_dia: float = 28.0          # [DESIGN] inside the bolt heads (r 18 - 2.75)
    hub_head_room: float = 3.25         # [DESIGN] the core's length: the heads (3) + clear, between the plate and the flange
    hub_flange_t: float = 4.0           # [DESIGN]
    hub_flange_dia: float = 42.0        # [DESIGN] = WristRollParams.face_dia
    hub_bore: float = 10.0              # [DESIGN] = WristRollParams.bore: the cable
    notch_r: float = 3.5                # [DESIGN] the flange's notch over each plate hole: the screw's head (r 2.75) + clear, for its key
    bolt_circle_dia: float = 36.0       # [DESIGN] 4x M3 on it ...
    bolt_deg: tuple = (30.0, 150.0, 210.0, 330.0)   # [DESIGN] ... from the plate's x axis: outside its +/- 11.5 flats (x +/-15.6), a
    #                                     key reaches each head along Y past the plate
    bolt_len: float = 10.0              # [DESIGN] M3 x 10: through the flange into its nut in the pulley's face, the tip 2 pitches past
    screw: ShcsSize = M3_SHCS           # [DATASHEET] ISO 4762 M3 (tools/bom.py EXTRAS)

    @property
    def hub_len(self) -> float:
        """The plate's +Y face to the pulley's face (DEFAULT)."""
        return self.hub_head_room + self.hub_flange_t


@dataclass(frozen=True)
class WristConfig:
    plate: PlateParams = PlateParams()
    seat: SeatParams | None = SeatParams()       # LEGACY: j3_coupler's seat; DEFAULT: none, the plate carries the flange
    flange: FlangeParams | None = None           # DEFAULT: the wrist_pitch output flange on the plate
    tower: TowerParams = TowerParams()
    end_face: EndFaceParams = EndFaceParams()
    roll: WristRollParams | None = None          # DEFAULT: the wrist roll drive's frame - the plate only its wall, no tower
    connector: ConnectorParams = ConnectorParams()
    pitch_inset: float = 0.0                     # the part and all it carries this far along its +Z from the capture (PITCH_INSET)


LEGACY = WristConfig()     # the SolidWorks part, exactly

# What the part builds: the wrist roll drive's frame round the pancake and the tower round R' (WristRollParams) - the
# plate only its wall on the forearm's side, nothing past it -, the wrist_pitch output flange on that wall (j3_coupler#2
# and its seat gone) - the old tower, its end face and gripper_clamp_bracket gone; the gripper's connector bolted to the
# roll pulley's face.
DEFAULT = replace(LEGACY, seat=None, flange=FlangeParams(), roll=WristRollParams(),
                  connector=replace(LEGACY.connector, hub=True), pitch_inset=PITCH_INSET)   # [DESIGN]
