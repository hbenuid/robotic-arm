"""GripperConfig - every dimension of the gripper (assemblies/gripper.py): its four measured parts, PapaBravo's "Rack &
Pinion Robotic Gripper Jaw" (Printables, licensed CC BY 4.0 - attribution required), each in its export's frame; the
MG996R as the vendor model has it (ServoParams), the cage designed here (CageParams) and where the module puts them all
(LayoutParams; lib/gripper/layout.py).

- BaseParams - gripper_base: the frame the MG996R hangs on - a plate with the jaws' two slots, a long wall each side and
  a rib inside each (the racks run between them), two posts the servo's tabs screw onto (each a block flared out to the
  tab, its two screw holes down from the top, a nut trap from the outer face, a slot across the top for the tab's rib).
  Frame: the plate's bottom on z = 0, centred on x and y, the posts toward +z.
- RackParams - gripper_rack: a bar with its teeth along one long edge (toward -y) and two split posts on its top (+z)
  that run in the base's slot; an M3 through each post, its head sunk in the bar's bottom.
- JawParams - gripper_jaw: a flat arm under the base, two nut bosses on it (the rack's screws), and at its +x end the
  foot - a chamfered riser - with the finger standing on it (+z): its gripping face flat (finger_x, facing -x), its back
  a tilted half-round, its top rounded across.
- PinionParams - gripper_pinion: a 14-tooth involute spur on the axis (z), the M3's head recess from its bottom, the
  servo spline's press bore from its top.

Every number was measured 2026-11-07 on the STEP exports (lib/reference.py MEASURED: Base.step, Rack.step, Jaw.step,
'SpurGear-14 teeth.step' in rack-pinion-robotic-gripper-jaw-model_files.zip): [REFERENCE] unless tagged - including the
export's own few hundredths of asymmetry (the rack's and the jaw's two screw positions); ServoParams on the vendor model;
CageParams and LayoutParams [DESIGN] where tagged. Units mm. Frozen dataclasses;
variants via dataclasses.replace. Pure Python: the builders are lib/gripper/body.py.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class BaseParams:
    half_x: float = 40.0
    half_y: float = 18.0
    plate_t: float = 4.0
    wall_y: float = 16.2                               # the long walls' inner face (|y|; their outer face: half_y)
    wall_top: float = 9.0                              # the walls' and the ribs' top
    rib_y: tuple[float, float] = (4.6, 6.1)            # the ribs (|y|): the racks run between a rib and a wall ...
    rib_x: float = 10.0                                # ... each rib from |x| = rib_x to the plate's end: the pinion between
    slot_y: tuple[float, float] = (10.175, 14.575)     # the slots through the plate the racks' posts run in (|y|) ...
    slot_x: float = 33.9                               # ... straight to |x| = slot_x ...
    slot_end_r: float = 2.1                            # ... then a disc at each end, centred on (slot_x, slot_end_y)
    slot_end_y: float = 12.375
    posts_x: tuple[tuple[float, float], ...] = ((-15.5, -9.25), (31.25, 37.5))   # the servo's posts, x from .. to
    post_y: float = 5.0                                # each post's lower block (|y|), from the plate to post_mid_z ...
    post_mid_z: float = 13.0
    post_top_y: float = 9.5                            # ... then flared out to |y| = post_top_y ...
    flare_deg: float = 40.0                            # ... at this angle off the vertical, then straight up ...
    post_top_z: float = 22.0                           # ... to the top, where the servo's tabs sit
    holes_x: tuple[float, float] = (-13.0, 35.0)       # the tab screws' holes (M3 clearance), two per post at |y| = hole_y,
    hole_y: float = 5.0                                # from hole_z up through the top
    hole_r: float = 1.6
    hole_z: float = 9.0
    nut_y: tuple[float, float] = (2.2, 7.8)            # each screw's nut trap (|y|), from nut_z[0] to nut_z[1], open on
    nut_z: tuple[float, float] = (17.75, 20.25)        # the post's outer face, to x = nut_x (one per post)
    nut_x: tuple[float, float] = (-9.9, 31.9)
    rib_slot_y: float = 1.0                            # the slot across each post's top for the servo tab's rib (|y|),
    rib_slot_z: float = 20.5                           # from rib_slot_z to the top, from the post's inner face to rib_slot_x
    rib_slot_x: tuple[float, float] = (-14.75, 36.75)


@dataclass(frozen=True)
class RackParams:
    half_x: float = 28.0
    body_y: tuple[float, float] = (8.75, 16.0)         # the bar; its teeth stand on body_y[0], toward -y
    t: float = 6.0                                     # the bar and its teeth, z 0 .. t
    teeth: int = 17                                    # centred on x = 0, pitch apart
    pitch: float = 3.31
    tip_y: float = 6.2955                              # a tooth (each side of its centre line): the tip flat to tip_x,
    tip_x: float = 0.167                               # a round of tip_r (centre (tip_x, tip_y + tip_r)) onto ...
    tip_r: float = 0.4855
    flank_x: float = 0.6525                            # ... the straight flank at flank_x, up to flank_y ...
    flank_y: float = 7.2573
    arc1_r: float = 1.6807                             # ... an arc (centre (arc1_x, flank_y)) to where it touches (y 7.515) ...
    arc1_x: float = 2.3332
    arc2_r: float = 4.5222                             # ... an arc round it (centre arc2_c) down to the root on body_y[0]
    arc2_c: tuple[float, float] = (5.1412, 6.8218)
    posts_x: tuple[tuple[float, float], ...] = ((-24.39, -16.39), (16.41, 24.41))   # the two posts: round-ended
    post_y: tuple[float, float] = (10.375, 14.375)     # (the ends' radius half the post's width), from t to post_top
    post_top: float = 10.3
    slits_x: tuple[tuple[float, float], ...] = ((-21.0, -19.78), (19.78, 21.0))   # each post split across, from slit_z
    slit_z: float = 6.1
    holes_x: tuple[float, float] = (-20.39, 20.41)     # the M3 through each post (centre y: the post's) ...
    hole_r: float = 1.65
    head_r: float = 3.1                                # ... its head sunk in the bar's bottom, head_z deep
    head_z: float = 2.8


@dataclass(frozen=True)
class JawParams:
    arm_x: tuple[float, float] = (-28.0, 28.0)         # the arm (z 0 .. t), its corners rounded corner_r
    arm_y: tuple[float, float] = (8.75, 16.0)
    t: float = 3.0
    corner_r: float = 1.0
    foot_x: tuple[float, float] = (11.5, 28.0)         # the foot under the finger: the plate widened to foot_y ...
    foot_y: tuple[float, float] = (-8.0, 16.0)
    riser_y: float = 8.0                               # ... and on it a riser (|y| <= riser_y at the plate), chamfered at
    riser_top: float = 6.0                             # 45 deg on its -y, +y and +x sides up to riser_top
    finger_x: float = 11.5                             # the finger's flat gripping face (facing -x), the riser's -x face too
    finger_w: float = 5.0                              # the finger's half-width (|y|), and its back's radius
    finger_top: float = 45.0
    top_r: float = 4.0                                 # its top's two long edges rounded (centres |y| finger_w - top_r)
    back_point: tuple[float, float, float] = (20.1132, 0.0, 4.1251)   # its back: a cylinder of finger_w round this axis,
    back_dir: tuple[float, float] = (-0.2009, 0.9796)                 # leaning toward the face (x, z of its direction)
    bosses: tuple[tuple[float, float], ...] = ((-20.3901, 12.3735), (20.4076, 12.375))   # the nut bosses (centres) ...
    boss_size: tuple[float, float] = (8.0, 7.0)        # ... round-cornered (boss_r), from t to riser_top, a hex through
    boss_r: float = 2.0                                # each (nut_af across flats, flats along x), an M3 hole under it
    nut_af: float = 5.8
    hole_r: float = 1.6
    notch_x: tuple[float, float] = (25.0, 28.0)        # a ramp cut under the foot's end: 0 deep at notch_x[0], notch_depth
    notch_y: tuple[float, float] = (-1.0, 6.0)         # at the end
    notch_depth: float = 2.0


@dataclass(frozen=True)
class PinionParams:
    teeth: int = 14                                    # a tooth centred on +x
    module: float = 1.02
    pressure_deg: float = 14.5                         # [REFERENCE] a standard 14.5 deg involute: with thickness, the fit to the
    thickness: float = 1.528                           # export's outline (2-D symmetric difference 0.018 mm^2) - the tooth's
                                                       # arc thickness on the pitch circle, thinned for backlash
    tip_r: float = 8.16
    root_r: float = 5.8652
    fillet_r: float = 0.6                              # the root's rounds, onto the radial flank under the base circle
    width: float = 6.5
    head_r: float = 3.1                                # the M3's head recess from the bottom (z 0) ...
    head_z: float = 2.4
    hole_r: float = 1.6                                # ... its hole through ...
    spline_r: float = 2.85                             # ... and the servo spline's press bore from spline_z to the top
    spline_z: float = 3.3

    @property
    def pitch_r(self) -> float:
        return self.teeth * self.module / 2.0


@dataclass(frozen=True)
class ServoParams:
    """[REFERENCE] the MG996R as the repo's vendor model has it (vendor/mg996r_servo.step, measured 2026-11-07), in
    its frame: tabs along X, height along Y (the back y 0, the spline's end spline_top), thickness along Z."""
    spline_x: float = -10.25           # the spline's axis (z 0)
    spline_top: float = 45.2
    case_x: tuple[float, float] = (-20.5, 20.5)   # the case's ends - 41.0 long; the datasheet's 40.7 (lib/params.py
    #                                               MG996R_BODY_L), the base's posts 40.5 apart: a 0.1 press a side
    tab_face: float = 31.2             # the tabs' spline-side face: it sits on the base's posts' tops
    tab_t: float = 2.4                 # the tabs' thickness (their back face at tab_face - tab_t)
    half_t: float = 10.25              # the case's half-thickness (|z|)


@dataclass(frozen=True)
class CageParams:
    """[DESIGN] gripper_cage - the bracket designed here (no reference): a back plate on the servo's back that bolts onto
    gripper_j3_connector's plate (4x M3 x 8 from the connector's flange side through its plate's holes, into nuts in
    slots from the back plate's edges), two side walls along the servo's case, and an ear on each pair of the servo's tabs
    over the base's posts - the tab screws (M3) clamp ear, tab and post together into the posts' nut traps, put in
    through the back plate's access holes before the cage goes onto the connector. In the base's frame (BaseParams)."""
    back_gap: float = 0.2              # the back plate off the servo's back
    back_t: float = 5.0                # its thickness: the connector screws' nuts sit in its middle
    back_x: tuple[float, float] = (-22.0, 37.5)   # its extent along the jaws (x): over the posts and the connector's holes
    wall_gap: float = 0.2              # the side walls off the case's sides ...
    wall_t: float = 2.0                # ... this thick, from the ears up to the back plate
    ear_t: float = 3.0                 # each ear on its pair of tabs, over the post, from the post's outer face ...
    ear_inner_gap: float = 0.375       # ... to this short of the case's end
    screw_hole: float = 3.4            # the tab screws' and the connector screws' clearance (lib/fasteners.py M3_CLEAR)
    access_dia: float = 6.5            # the back plate's hole over each tab screw: its head and the key
    nut_af: float = 5.6                # the connector screws' nut slots: M3 (5.5 across flats) + fit ...
    nut_h: float = 2.6                 # ... 2.4 thick + fit, centred in back_t
    cable_w: float = 4.5               # the servo cable's slot in the back plate, from cable_x0 to the axis ...
    cable_x0: float = -12.0
    bore: float = 10.0                 # ... and a hole on the axis for its plug: the connector's bore


@dataclass(frozen=True)
class LayoutParams:
    """Where the module puts its parts, in the base's frame (BaseParams): the servo upside down on the posts' tops (its
    case between them, so its spline - the pinion's axis - at x0()), the pinion pinion_gap off the plate, each rack on the
    plate (its posts down through the slot), each jaw under the plate on its rack's posts' tips; the racks centred at
    zero - the jaws travel_x either way."""
    pinion_gap: float = 0.25           # [DESIGN] the pinion's bottom off the plate (pushed on the spline that far)
    travel_x: float = 10.5             # [DESIGN] each jaw's travel from zero: a rack's post's round end stops in its slot's
    #                                    end at 10.84 opening, 10.86 closing (the jaws' gap 1.28 .. 44.68) - 0.34 short of
    #                                    that (gap 2.0 .. 44.0; tests/gripper/test_module.py)


@dataclass(frozen=True)
class GripperConfig:
    base: BaseParams = field(default_factory=BaseParams)
    rack: RackParams = field(default_factory=RackParams)
    jaw: JawParams = field(default_factory=JawParams)
    pinion: PinionParams = field(default_factory=PinionParams)
    servo: ServoParams = field(default_factory=ServoParams)
    cage: CageParams = field(default_factory=CageParams)
    layout: LayoutParams = field(default_factory=LayoutParams)


DEFAULT = GripperConfig()
