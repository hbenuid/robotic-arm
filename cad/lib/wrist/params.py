"""WristConfig - every dimension of the wrist body, wrist_link (the wrist_pitch stage: j3_coupler#2's flange bolts to
its plate on the pitch axis), in its part frame, of the wrist roll drive it carries (WristRollParams) and of the
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
tests/test_reference_match.py); DEFAULT is what the part builds: the plate's round end on the pitch axis with its
seat, and on its underside (the free face, away from the forearm) the wrist roll drive's frame - the pancake ON the
pitch axis, the roll axis R' beside it (WristRollParams; docs/wrist_roll.md); the tower, its end face and the bracket
are gone (lib/placements.py SHIFTS moves the gripper onto R').

Every number below was measured on the reference 2026-09-27 (face / vertex census; tests/wrist/test_wrist_link.py
re-checks the builds against it): [REFERENCE] unless tagged. SolidWorks' float noise (<= 6e-4) is rounded off the end
face's holes and the block. Units mm. Frozen dataclasses; variants via dataclasses.replace.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from lib.bearings import BEARING_6806_BORE, BEARING_6806_OD, BEARING_6806_SHOULDER_DIA, BEARING_6806_WIDTH
from lib.belts import GT2_PULLEY_20T_TEETH, centre_distance
from lib.fasteners import M3_CLEAR, M3_NUT, M3_SHCS, NutSize, ShcsSize
from lib.motors import MKS_SERVO42D_STACK, MKS_SERVO42D_W


@dataclass(frozen=True)
class PlateParams:
    """The plate, z 0..thickness: a round end on the pitch axis, flats at y = +/-half_width, a round end on the origin
    (the tower's wall). SolidWorks drew the pitch end 0.0007 over the flats' half width: the flats clip it."""

    thickness: float = 5.0
    pitch_x: float = -45.445774        # the wrist_pitch axis (x, 0), j3_coupler#2's
    pitch_r: float = 39.000699         # the round end on it
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

    @property
    def body_length(self) -> float:
        """Its rear face to its mounting face (23)."""
        return self.face_z - self.rear_z


@dataclass(frozen=True)
class WristRollParams:
    """The wrist roll drive (assemblies/wrist_roll_drive.py; docs/wrist_roll.md), the forearm roll drive's pattern one
    joint further out: the pancake ON the wrist_pitch axis, on the plate's free face (the side away from the forearm),
    belted to the roll axis beside it - centre_distance across in the wrist's swing plane, at the motor's height - so the
    wrist roll axis R' no longer meets the pitch axis (a wrist offset: the wrist is no longer spherical). wrist_link IS
    the frame (the stator): the plate (j3_coupler#2's seat) and on its free face one body round the motor - its pocket
    open on the +X face, its front wall the plate the pancake bolts to (slotted along Y: the belt's tension), no wall
    behind it: a window there takes the pancake's MKS board, on its rear face - and the tower round R' (the 6806 pair
    back to back on a lip, a bearing boss on the front face, the shaft's bay behind bearing 1 open on +X - the gripper
    servo's cable leaves the rotor's bore there - with the hard stop's post). The rotor: the
    PULLEY (wrist_roll_pulley: its hub through bearing 2, the 40T band, the face gripper_j3_connector bolts to - the
    roll's output) and the SHAFT (wrist_roll_shaft: its hub through bearing 1, its flange in the bay with the rotor
    clamp's nuts and the stop lug); the clamp's 4x M3 from the pulley's face through both hubs hold them and both inner
    rings as one.

    MODULE FRAME: origin on R' at the pitch axis' station; +Z along R' toward the gripper (wrist_link's +X); +X away from
    the plate (the open face, wrist_link's -Z); +Y = wrist_link's +Y; R' lies `side` x centre_distance along it from the
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
    # the motor: centred on the pitch axis (its body's middle at module z 0), its plug spun to +X (the open face)
    pancake: PancakeParams = PancakeParams()
    motor_spin_deg: float = 90.0        # [DESIGN] about its axis: the plug (the vendor frame's -Y) -> module +X
    pocket_clear: float = 0.75          # [DESIGN] round the body in the pocket, along Y past the tension travel
    pocket_rear: float = 1.0            # [DESIGN] behind its rear face (past motor_drop)
    motor_drop: float = 1.25            # [DESIGN] how far back the pancake slides in from the open face: its pilot behind the plate + 0.25
    # its MKS SERVO42D on its rear face (the board's encoder reads the shaft's rear end), turned with it (motor_spin_deg:
    # its motor plug toward the pancake's, at the open face; the side plugs along Y), in a window through the frame's
    # rear - the board's housing square + the tension travel + board_clear, open on +X like the pocket: they go in together
    board_w: float = MKS_SERVO42D_W     # [REFERENCE] 43: the board's housing square, from the pancake's rear face back
    board_stack: float = MKS_SERVO42D_STACK   # [REFERENCE] 14.1: the board behind the pancake's rear face
    board_clear: float = 0.45           # [DESIGN] round the board's housing in the window, along Y past the tension travel: the
    #                                     window's wall to the shaft's bay keeps 1.5 (pocket_clear's 0.75 would leave 1.23)
    shaft_slit: float = 6.0             # [DESIGN] the Ø5 shaft's slit through the plate, from the pilot slot out through the open face
    pad_t: float = 4.0                  # [DESIGN] the plate (the pocket's front wall) the pancake's 4x M3 bolt through
    pad_slot_len: float = 5.0           # [DESIGN] +/- 2.5 along Y: the belt's tension (no idler)
    pad_bolt_dia: float = 3.4
    pad_pilot_w: float = 22.3           # [DESIGN] the pilot's slot
    pulley_seat: float = 0.5            # [DESIGN] the 20T's hub face over the pilot (the forearm roll's)
    t20_hub: float = 10.95              # [REFERENCE] vendor 20T: its tooth band's centre from its hub face (7.45 + 3.5)
    # the frame round the motor and the tower: its outline seen along Z the hull of a circle about R' and the pocket's box,
    # from the plate's free face to the open +X face
    frame_r: float = 30.0               # [DESIGN] the outline's circle about R': the open face (x frame_r) and the tower's far side
    pocket_wall: float = 2.5            # [DESIGN] round the pocket's far side (behind it the board's window: no wall)
    bearing_boss_dia: float = 52.0      # [DESIGN] round R' on the front face, bearing 2's seat in it (the forearm roll's)
    bay_r: float = 22.6                 # [DESIGN] the shaft's bay behind bearing 1, open on +X: the stop lug's corners + 1; bearing 1 (r 21)
    #                                     passes through it with the shaft
    bay_clear: float = 1.0              # [DESIGN] the bay's floor behind the shaft's rear face by its hub's length + this (it slides in)
    tower_wall: float = 2.0             # [DESIGN] behind the bay's floor
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
    seat: SeatParams = SeatParams()
    tower: TowerParams = TowerParams()
    end_face: EndFaceParams = EndFaceParams()
    roll: WristRollParams | None = None          # DEFAULT: the wrist roll drive's frame on the plate, no tower
    connector: ConnectorParams = ConnectorParams()


LEGACY = WristConfig()     # the SolidWorks part, exactly

# What the part builds: the plate with j3_coupler#2's seat and, on its free face, the wrist roll drive's frame round the
# pancake and the tower round R' (WristRollParams) - the old tower, its end face and gripper_clamp_bracket gone; the
# gripper's connector bolted to the roll pulley's face.
DEFAULT = replace(LEGACY, roll=WristRollParams(), connector=replace(LEGACY.connector, hub=True))   # [DESIGN]
