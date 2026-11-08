"""Every fit where a bought part presses, slides or drops into a printed one - GAUGES: one gauge per kind of fit, each
size it tests read from the constant the part's builder sizes it with, so a gauge follows a changed fit. A new fit adds
its Use here (to the gauge of its kind, or a gauge of its own); tests/test_gauges.py checks the list.

Fits that share a bought part merge into one gauge (the sweep runs through all their values, lib/gauges/coupons.py
sweep()); several parts with the same size are one Use each, so each keeps reading its own constant. Left out: the
clearances of 0.5 mm a side or more, stops and shoulders, the belt's tooth mesh, printed-into-printed fits (tools/
AGENTS.md export_gauges.py says what each gauge prints and how to use it). Units mm: a diameter, an across-flats, a gap.
"""
from __future__ import annotations

from dataclasses import dataclass

from lib.base.params import DEFAULT as BASE
from lib.cycloidal.layout import ring_pin_hole_dia
from lib.cycloidal.params import DEFAULT_CONFIG as DRIVE
from lib.fasteners import M3_CLEAR, M4_CLEAR, M4_NUT, M5_CLEAR, M5_NUT
from lib.forearm.params import DEFAULT as FOREARM
from lib.gauges.coupons import REACH, STEP, sweep
from lib.gripper.params import DEFAULT as GRIPPER
from lib.pulley.params import DEFAULT as PULLEY
from lib.upper_arm.params import DEFAULT as UPPER_ARM
from lib.wrist.params import DEFAULT as WRIST
from lib.yaw_coupler.layout import COUNTERBORE as FORK_COUNTERBORE
from lib.yaw_coupler.layout import NUT_SLOT as FORK_NUT_SLOT
from lib.yaw_coupler.params import DEFAULT as YAW_COUPLER


@dataclass(frozen=True)
class Use:
    """One size the arm prints for a fit, and where (the parts; press / slide / clearance as the code says)."""
    value: float
    where: str


@dataclass(frozen=True)
class Gauge:
    """One kind of fit: its coupon builder (lib/gauges/coupons.py BUILDERS) with its options, the debossed title, the
    arm's sizes. `step` / `reach`: the sweep (the big rings: 0.10, one either side)."""
    key: str
    group: str
    name: str
    title: str
    coupon: str
    uses: tuple
    options: tuple = ()
    step: float = STEP
    reach: int = REACH
    note: str = ""

    @property
    def values(self) -> tuple:
        return tuple(round(u.value, 3) for u in self.uses)

    @property
    def sizes(self) -> list[float]:
        return sweep(self.values, self.step, self.reach)


BIG = {"step": 0.10, "reach": 1}   # [DESIGN] the rings from Ø60 up: 0.10 apart, one either side

_D, _TOL = DRIVE, DRIVE.tolerances
_ROLL, _WROLL = FOREARM.drive, WRIST.roll
_GB, _GC = GRIPPER.base, GRIPPER.cage
_POSTS = (GRIPPER.base.posts_x[0][1], GRIPPER.base.posts_x[1][0])   # the servo posts' inner faces

GROUPS = (
    ("seats", "Bearings: outer race into a printed seat"),
    ("journals", "Bearings: printed journal into the inner race"),
    ("pins", "Pins, shafts and pilots"),
    ("screws", "Screws"),
    ("nuts", "Nuts"),
    ("servo", "Servo"),
)

GAUGES = (
    # --- bearings: an outer race into a printed seat
    Gauge("seat90", "seats", "Ø90 seats: the 6814s' outer races, the thrust washers and cage", "Ø90 SEAT", "ring_seats", (
        Use(_D.housing.output_bearing_seat_dia, "the 6814 seats: cycloidal_shell_ring, j1_link (press)"),
        Use(2.0 * BASE.cap.groove_r[1], "the thrust groove's outer wall: base (AXK / AS 6590)"),
        Use(YAW_COUPLER.hub.recess_dia, "the upper thrust washer's recess: j1_coupler")),
        (("band", 3.0),), note="3 mm seat over a 1 mm lip, 3 mm wall.", **BIG),
    Gauge("seat42", "seats", "Ø42 seats: the 6806s' outer races", "6806 SEAT", "seat_strip", (
        Use(UPPER_ARM.elbow.bore_dia, "j1_link's upper elbow seat (line to line)"),
        Use(_ROLL.bearing_od + _ROLL.seat_add, "forearm_roll_block's tower (press)"),
        Use(_WROLL.bearing_od + _WROLL.seat_add, "wrist_link's tower (press)"),
        Use(BASE.bore.upper_dia, "base (press)"),
        Use(UPPER_ARM.elbow.seat_dia, "j1_link's lower elbow seat (press)"),
        Use(FOREARM.boss.seat_dia, "j2_link's wrist boss (press)")),
        (("band", 5.0),), note="5 mm seat (the bearing is 7) over a 1 mm lip: push it back out from below."),
    Gauge("seat35", "seats", "Ø35 seat: the 6003s' outer race", "6003 SEAT", "seat_strip", (
        Use(_D.disc.center_bore_dia, "cycloidal_disc_1, _2: the centre bore"),),
        (("band", 5.0),), note="5 mm seat over a 1 mm lip."),
    Gauge("seat16", "seats", "Ø16 pocket: the 625's outer race", "625 SEAT", "hole_strip", (
        Use(_D.bearings.inp_od + _TOL.bearing_seat_bore_add, "cycloidal_output_hub (blind, slip to light press)"),),
        (("depth", _D.bearings.inp_width), ("width", 16.4), ("push", 10.0)),
        note="As deep as the bearing, a Ø10 hole under it to push it back out."),
    # --- bearings: a printed journal into an inner race
    Gauge("boss70", "journals", "Ø70 journals: the 6814s' inner races", "6814 BORE", "ring_bosses", (
        Use(_D.shell.sleeve_od, "the motor plate's sleeve: the 6814 slides on"),
        Use(_D.output_hub.od, "the output hub's grip, the sleeve's last 10 mm (press)")),
        (("band", 4.0),), note="A 4 mm band, 3 mm wall; the bearing goes down over it.", **BIG),
    Gauge("boss65", "journals", "Ø64.8 ring: the thrust washers' and cage's bore", "THRUST BORE", "ring_bosses", (
        Use(2.0 * BASE.cap.groove_r[0], "the base's seat ring (clearance)"),),
        (("band", 3.4),), note="A band as tall as the base's ring.", **BIG),
    Gauge("boss30", "journals", "Ø30 journals: the 6806s' inner races", "6806 BORE", "peg_strip", (
        Use(_ROLL.neck_dia, "forearm_roll_pulley's neck: the bearing slides over"),
        Use(PULLEY.hub_dia, "the 90T / 120T hubs (line to line)"),
        Use(YAW_COUPLER.hub.stub_dia, "j1_coupler's stub (line to line)"),
        Use(_ROLL.stub_dia, "forearm_roll_block's elbow stub (line to line; printed sideways)"),
        Use(WRIST.flange.stub_dia, "wrist_link's stub (line to line)"),
        Use(_ROLL.bearing_bore + _ROLL.journal_add, "forearm_roll_pulley, forearm_roll_shaft (press)"),
        Use(_WROLL.bearing_bore + _WROLL.journal_add, "wrist_roll_pulley, wrist_roll_shaft (press)")),
        (("height", 5.0), ("spacing", 37.0), ("bore", 24.0)), note="Rings 5 mm tall, a 6806 over one at a time."),
    Gauge("boss17", "journals", "Ø17 journal: the 6003s' inner race", "6003 BORE", "peg_strip", (
        Use(_D.shaft.bearing_seat_od, "cycloidal_eccentric_shaft's lobes (light press)"),),
        (("height", 5.0), ("spacing", 27.0)), note="Pegs 5 mm tall, a 6003 over one at a time."),
    # --- pins, shafts and pilots
    Gauge("pin4", "pins", "Ø4 pins: the ring pins and output pins", "4MM PIN", "hole_strip", (
        Use(ring_pin_hole_dia(_D), "cycloidal_shell_ring, j1_link (ring pins), the output hub (output pins): blind"),),
        (("depth", 8.0), ("width", 4.3)), note="Blind, 8 deep."),
    Gauge("pin5", "pins", "Ø5 support pin", "5MM PIN", "hole_strip", (
        Use(_D.shaft.support_pin_dia + 2.0 * _TOL.dowel_bore_clearance_add, "cycloidal_eccentric_shaft (firm press)"),),
        (("depth", _D.shaft.support_pin_hole_depth), ("width", 5.3)), note="Blind, as deep as the shaft's."),
    Gauge("dbore", "pins", "The motor shaft's D-bore", "NEMA D-BORE", "hole_strip", (
        Use(_D.shaft.d_bore_dia + 2.0 * _TOL.d_bore_clearance_add, "cycloidal_eccentric_shaft (slip to light press)"),),
        (("depth", 10.0), ("width", 5.3), ("shape", "d"), ("floor", 0.0), ("through", True),
         ("d_flat", _D.shaft.d_bore_flat / 2.0 + _TOL.d_bore_clearance_add)),
        note="Through, 10 tall; the flat moves out half a step with each step."),
    Gauge("pilot", "pins", "The NEMA 17 pilot Ø22", "NEMA PILOT", "hole_strip", (
        Use(_D.motor.pilot_dia + 2.0 * _TOL.mating_surface_add, "cycloidal_motor_plate's recess"),
        Use(BASE.motor.holes.pilot_dia, "base_motor_mount"),
        Use(_ROLL.motor_holes.pilot_dia, "forearm_roll_block"),
        Use(FOREARM.holes.pilot_dia, "j2_link"),
        Use(_WROLL.pad_pilot_w, "wrist_link: the slot's width")),
        (("depth", 3.0), ("width", 22.4), ("floor", 0.0), ("through", True)),
        note="Through a 3 mm plate (the pilot stands 2 proud)."),
    Gauge("gbox", "pins", "The EG17-G5 gearbox's Ø25 boss", "GEARBOX BOSS", "hole_strip", (
        Use(UPPER_ARM.arm.seat_boss_dia, "j1_link's web"),),
        (("depth", UPPER_ARM.arm.seat_boss_depth), ("width", 25.6), ("under", UPPER_ARM.arm.seat_bore_dia)),
        note="As deep as j1_link's, the shaft's hole under it."),
    Gauge("spline", "pins", "The MG996R's output spline", "SERVO SPLINE", "hole_strip", (
        Use(2.0 * GRIPPER.pinion.spline_r, "gripper_pinion (press: the spline cuts its own)"),),
        (("depth", GRIPPER.pinion.width - GRIPPER.pinion.spline_z), ("width", 5.8)), note="Blind, as deep as the pinion's."),
    Gauge("magnet", "pins", "The 3 x 2 magnets' pockets", "MAGNET 3X2", "hole_strip", (
        Use(_ROLL.back.pocket_dia, "forearm_roll_block, forearm_roll_back_panel (glued)"),
        Use(_WROLL.back.pocket_dia, "wrist_link, wrist_back_panel (glued)")),
        (("depth", _ROLL.back.pocket_depth), ("width", 3.3)), note="Blind, as deep as the panels'."),
    # --- screws
    Gauge("m2", "screws", "M2 clearance holes", "M2 HOLE", "hole_strip", (
        Use(BASE.home.board_hole_dia, "yaw_home_bracket: the KY-003 board's screws"),),
        (("depth", 3.0), ("width", 2.5), ("floor", 0.0), ("through", True)), note="Through 3."),
    Gauge("m3", "screws", "M3 clearance holes", "M3 HOLE", "hole_strip", (
        Use(2.0 * _GB.hole_r, "gripper_base, gripper_jaw, gripper_pinion (PapaBravo's)"),
        Use(2.0 * GRIPPER.rack.hole_r, "gripper_rack (PapaBravo's)"),
        Use(M3_CLEAR, "the motor screws, the rotor clamps, the connector, gripper_cage, the home bracket")),
        (("depth", 4.0), ("width", 3.5), ("floor", 0.0), ("through", True)), note="Through 4."),
    Gauge("m3s", "screws", "M3 clearance holes, sideways", "M3 HOLE SIDE", "side_strip", (
        Use(2.0 * _GB.hole_r, "gripper parts (PapaBravo's)"), Use(2.0 * GRIPPER.rack.hole_r, "gripper_rack"),
        Use(M3_CLEAR, "every other M3")),
        (("depth", 4.0),), note="The same holes printed sideways."),
    Gauge("m4", "screws", "M4 clearance holes", "M4 HOLE", "hole_strip", (
        Use(M4_CLEAR, "the housing, fork, pulley and gearbox screws"),),
        (("depth", 5.0), ("width", 4.5), ("floor", 0.0), ("through", True)), note="Through 5."),
    Gauge("m4s", "screws", "M4 clearance holes, sideways", "M4 HOLE SIDE", "side_strip", (
        Use(M4_CLEAR, "the base's and base_motor_mount's posts, the motor plate's sleeve (printed sideways)"),),
        (("depth", 5.0),), note="The same holes printed sideways."),
    Gauge("m5", "screws", "M5 idler axles", "M5 HOLE", "hole_strip", (
        Use(M5_CLEAR, "the idler spacers' bores, the idler slots"),),
        (("depth", 5.0), ("width", 5.6), ("floor", 0.0), ("through", True)), note="Through 5."),
    Gauge("m3head", "screws", "M3 head counterbores", "M3 HEAD", "hole_strip", (
        Use(_D.motor.motor_bolt_head_dia + _TOL.bolt_clearance_add, "cycloidal_motor_plate"),
        Use(BASE.motor.holes.pocket_dia, "base_motor_mount"),
        Use(_ROLL.motor_holes.pocket_dia, "forearm_roll_block"),
        Use(FOREARM.holes.pocket_dia, "j2_link")),
        (("depth", FOREARM.holes.screw.head_h), ("width", 5.8), ("floor", 2.0), ("under", M3_CLEAR)),
        note="A head deep over an M3 hole. The wider ones elsewhere (rotor clamps, gripper) follow if these fit."),
    Gauge("m4head", "screws", "M4 head counterbores", "M4 HEAD", "hole_strip", (
        Use(_D.housing.bolt_counterbore_dia, "cycloidal_shell_ring (housing bolts)"),
        Use(FORK_COUNTERBORE, "j1_coupler, j1_motor_leg (fork bolts)"),
        Use(PULLEY.counterbore[0], "gt2_pulley_90t, gt2_pulley_120t"),
        Use(UPPER_ARM.arm.seat_pocket_dia, "j1_link's web (gearbox screws)")),
        (("depth", 4.5), ("width", 7.5), ("floor", 2.0), ("under", M4_CLEAR)), note="4.5 deep over an M4 hole."),
    # --- nuts
    Gauge("m3slot", "nuts", "M3 nut slots, slid in from the side", "M3 NUT SLOT", "slot_strip", (
        Use(_ROLL.end_nut.af, "forearm_roll_pulley: the wall screws' nuts (line to line)"),
        Use(_ROLL.clamp_nut.af, "forearm_roll_shaft: the clamp's nuts (line to line)"),
        Use(_WROLL.nut.af, "wrist_roll_shaft: the clamp's nuts (line to line)"),
        Use(_GB.nut_y[1] - _GB.nut_y[0], "gripper_base's traps"),
        Use(_WROLL.nut.af + 2.0 * _WROLL.nut_fit, "wrist_roll_pulley's connector slots")),
        (("af", _ROLL.end_nut.af), ("height", _ROLL.end_nut.h + 0.2), ("screw", M3_CLEAR)),
        note="The size is the slot's width; a screw hole through each."),
    Gauge("m3hex", "nuts", "M3 nut hex pockets", "M3 NUT", "hole_strip", (
        Use(_GC.nut_af, "gripper_cage's back plate"),
        Use(BASE.home.nut.af + BASE.home.nut_clear, "base_motor_mount's plate: the home bracket's nuts (drop in)"),
        Use(GRIPPER.jaw.nut_af, "gripper_jaw (pressed)")),
        (("depth", _GC.nut_h), ("width", 6.8), ("shape", "hex"), ("floor", 1.6), ("under", M3_CLEAR)),
        note="A corner along the strip, an M3 hole under each."),
    Gauge("m4hex", "nuts", "M4 nut hex pockets", "M4 NUT", "hole_strip", (
        Use(_D.shell.sleeve_nut_af, "the motor plate's sleeve (press)"),
        Use(_ROLL.nut_af, "forearm_roll_block (press)"),
        Use(WRIST.flange.nut_af, "wrist_link (press)"),
        Use(YAW_COUPLER.hub.nut_af, "j1_coupler (press)"),
        Use(BASE.joint.nut_pocket_af, "the base's posts (press)"),
        Use(_D.housing.bolt_nut_pocket_af, "j1_link's housing nuts, the output hub (captive)")),
        (("depth", M4_NUT.h + 0.2), ("width", 8.4), ("shape", "hex"), ("floor", 1.6), ("under", M4_CLEAR)),
        note="Upright; the press size sideways too (next)."),
    Gauge("m4hexs", "nuts", "M4 nut hex pockets, sideways", "M4 NUT SIDE", "side_strip", (
        Use(BASE.joint.nut_pocket_af, "the base's posts (press)"),
        Use(_D.shell.sleeve_nut_af, "the motor plate's sleeve lugs (press)"),
        Use(_ROLL.nut_af, "forearm_roll_block's channels (press)")),
        (("depth", M4_NUT.h), ("shape", "hex"), ("blind", True), ("screw", M4_CLEAR)),
        note="A corner up, an M4 hole on through."),
    Gauge("m4slot", "nuts", "M4 nut slot", "M4 NUT SLOT", "slot_strip", (
        Use(M4_NUT.af + FORK_NUT_SLOT, "j1_coupler's disc (the fork's foot nuts)"),),
        (("af", M4_NUT.af), ("height", M4_NUT.h + FORK_NUT_SLOT), ("screw", M4_CLEAR)), note="The size is the width."),
    Gauge("m5slot", "nuts", "M5 nut channels (the idler axles)", "M5 NUT SLOT", "hole_strip", (
        Use(UPPER_ARM.idler.nut.af + UPPER_ARM.idler.nut_fit, "j1_link"),
        Use(BASE.idler.nut.af + BASE.idler.nut_fit, "base_motor_mount"),
        Use(FOREARM.idler.nut.af + FOREARM.idler.nut_fit, "j2_link"),
        Use(_ROLL.idler.nut.af + 2.0 * _ROLL.idler.nut_fit, "forearm_roll_block")),
        (("depth", M5_NUT.h + 0.2), ("width", 9.7), ("shape", "channel"), ("floor", 1.6), ("under", M5_CLEAR),
         ("channel", 2.0 * M5_NUT.af / 3.0 ** 0.5 + 0.4)),
        note="Open from the top, flats on the walls, an M5 hole under each."),
    # --- the servo
    Gauge("servo_len", "servo", "The MG996R's case between the posts", "MG996R LENGTH", "gap_clips", (
        Use(_POSTS[1] - _POSTS[0], "gripper_base's posts (press)"),),
        (("height", 8.0),), note="One U-clip per size."),
    Gauge("servo_w", "servo", "The MG996R's case between the cage's walls", "MG996R WIDTH", "gap_clips", (
        Use(2.0 * (GRIPPER.servo.half_t + _GC.wall_gap), "gripper_cage"),),
        (("height", 8.0),), note="One U-clip per size."),
)
