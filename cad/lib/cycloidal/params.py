"""DriveConfig - every dimension of the 20:1 cycloidal drive (the drive's single source of truth).

Ported from cycloidal_drive@2f1f67d src/params.py (see docs/cycloidal_drive.md). Ten frozen
parameter groups aggregated by a frozen ``DriveConfig``; variants via ``dataclasses.replace``:

    cfg = replace(DEFAULT_CONFIG, housing=replace(DEFAULT_CONFIG.housing, edge_chamfer=0.0))

Port adaptations (all listed in docs/cycloidal_drive.md "Port notes"):
  * the builders' hard-coded numbers are now tagged fields (``motor_plate_shaft_bore``,
    ``lip_radial``, ``ring_pin_entry_chamfer_*``, ``pillar_*_w``, ``bolt_nut_af``,
    ``bridge_flange_add``, ``pilot_height``, ``motor_bolt_thread_margin``, ``motor_bolt_recess``,
    ``bolt_clearance_add``);
  * unused fields were dropped (ProfileParams.spline_tolerance, PETGTolerances.bearing_inner_shaft_sub
    / sliding_clearance_add, HousingParams.wall_thickness / motor_plate_wall, BearingParams.ecc_qty /
    inp_qty);
  * the interface dimensions the arm needs are re-exported by lib/params.py (CYCLOIDAL_*).
Two configurations: LEGACY_CONFIG is the port (what the CadQuery exports in reference/cycloidal/ were built
from); DEFAULT_CONFIG is what the parts build - it differs only in HousingParams.bolt_count.
Units: mm, degrees where named *_deg. The stack-up datum (Z=0) is the OUTER face of the motor plate
(the NEMA 17 mounting face); +Z runs through the drive toward the output hub.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace


@dataclass(frozen=True)
class GearParams:
    """Core gear geometry - spec section 1.1 & 1.2."""

    num_lobes: int = 20
    num_ring_pins: int = 21          # N + 1
    eccentricity: float = 1.5        # mm
    ring_pin_circle_dia: float = 108.0
    ring_pin_dia: float = 4.0        # h6 ground steel dowel
    ring_pin_length: float = 35.0    # 3.5 mm motor-plate engagement + 28 mm bore zone + 3.5 mm bearing-zone wall

    @property
    def ring_pin_circle_radius(self) -> float:
        return self.ring_pin_circle_dia / 2.0

    @property
    def ring_pin_radius(self) -> float:
        return self.ring_pin_dia / 2.0

    @property
    def gear_ratio(self) -> int:
        """Reduction ratio = number of lobes (20:1)."""
        return self.num_lobes

    @property
    def disc2_phase_deg(self) -> float:
        """Disc-2 epitrochoid phase offset relative to disc 1, in degrees (-9).

        At orbit angle phi=pi (disc 2 sits opposite disc 1) the meshing kinematics require a disc
        rotation of -pi/N_lobes. It is baked into disc 2's PRINTED PROFILE (lib/cycloidal/disc.py),
        not applied as an assembly rotation, so the output-pin holes stay at the disc-local
        0/90/180/270 deg and line up with the stationary output pins after the (-e, 0) orbit
        translation. The two discs are therefore NOT interchangeable parts.
        """
        return -180.0 / self.num_lobes


@dataclass(frozen=True)
class DiscParams:
    """Cycloidal disc dimensions - spec section 1.2 & 1.3."""

    thickness: float = 10.0            # matches the 6003 bearing width
    center_bore_dia: float = 35.10     # 35 mm bearing OD + 0.10 mm clearance
    inter_disc_spacer: float = 2.0
    output_pin_count: int = 4
    output_pin_circle_dia: float = 60.0
    output_pin_dia: float = 4.0        # 4 x 45 mm h6 dowel, captured in blind hub holes
    output_pin_length: float = 45.0
    output_pin_hole_dia: float = 7.4   # 4 + 2*1.5 ecc + 0.4 clearance; 0.2 mm radial slack ~ +/-0.38 deg backlash
    lobe_chamfer: float = 1.0          # 45 deg chamfer on the outer epitrochoid edges (lead-in, elephant foot)


@dataclass(frozen=True)
class ShaftParams:
    """Eccentric shaft dimensions - spec section 1.4."""

    bearing_seat_od: float = 17.10     # slight clearance for the 6003 bore
    eccentricity: float = 1.5
    spine_od: float = 5.0              # shaft OD outside the lobe regions
    support_pin_dia: float = 5.0       # ground steel dowel (output-side 625 support)
    support_pin_length: float = 20.0
    support_pin_hole_depth: float = 11.0   # keeps a 1 mm wall to the 14 mm D-bore; pin 2 mm proud
    input_collar_od: float = 10.0      # enlarged input section around the D-bore
    d_bore_dia: float = 5.0            # motor shaft (clearance applied in the builder)
    d_bore_flat: float = 4.5           # D-flat width (matches the motor shaft D-cut)
    d_bore_depth: float = 14.0         # clears the 22 mm motor shaft past the 9 mm plate (13 + 1)
    bridge_flange_add: float = 6.0     # [DESIGN] bridge/retention flange OD = bearing_seat_od + this (23.10)

    @property
    def bridge_flange_od(self) -> float:
        return self.bearing_seat_od + self.bridge_flange_add


@dataclass(frozen=True)
class BearingParams:
    """All bearing dimensions - spec section 2."""

    ecc_bore: float = 17.0     # 6003-2RS eccentric bearings (x2)
    ecc_od: float = 35.0
    ecc_width: float = 10.0
    out_bore: float = 70.0     # 6814-2RS output bearings (x2)
    out_od: float = 90.0
    out_width: float = 10.0
    out_qty: int = 2
    inp_bore: float = 5.0      # 625-2RS eccentric-shaft support, output side (x1)
    inp_od: float = 16.0
    inp_width: float = 5.0


@dataclass(frozen=True)
class HousingParams:
    """Housing dimensions - spec section 5."""

    od: float = 140.0                  # sized for a 3 mm+ wall around counterbores / nut pockets
    bore_dia: float = 116.0
    edge_chamfer: float = 1.5          # 45 deg chamfer on the outer silhouette of both housing parts; 0 disables
    bolt_count: int = 8
    bolt_circle_dia: float = 125.0     # outside the 116 mm bore
    bolt_dia: float = 4.0              # M4
    bolt_length: float = 55.0          # M4 x 55 SHCS
    bolt_head_dia: float = 7.0
    bolt_head_height: float = 4.0
    bolt_counterbore_dia: float = 7.4  # head 7 + 0.4 clearance
    bolt_counterbore_depth: float = 4.5    # 4 mm head + 0.5 mm recess
    bolt_nut_af: float = 7.0           # [DATASHEET] M4 hex nut across flats (the nut model)
    bolt_nut_pocket_af: float = 7.2    # nut 7.0 AF + 0.2 pocket clearance
    bolt_nut_thickness: float = 3.2
    bolt_nut_depth: float = 4.0        # pocket depth (ring gear body output face + hub arm mount)
    output_bearing_seat_dia: float = 90.15   # press fit for the 6814 outer races
    motor_plate_shaft_bore: float = 15.0     # [DESIGN] motor-shaft pass-through bore in the motor plate
    lip_radial: float = 2.0            # [DESIGN] integral 6814 retention lip, radial thickness
    ring_pin_entry_chamfer_depth: float = 1.0   # [DESIGN] funnel at the bore/bearing-zone transition
    ring_pin_entry_chamfer_add: float = 1.0     # [DESIGN] funnel entry dia = pin hole dia + this
    pillar_inner_w: float = 18.0       # [DESIGN] reveal-window pillar tangential width at the bore
    pillar_outer_w: float = 10.0       # [DESIGN] ... and at the OD

    @property
    def lip_bore_dia(self) -> float:
        """Hub-clearance bore above the bearing seat (86.15)."""
        return self.output_bearing_seat_dia - 2.0 * self.lip_radial


@dataclass(frozen=True)
class MotorParams:
    """NEMA 17 motor dimensions - spec section 3.1."""

    shaft_dia: float = 5.0
    shaft_length: float = 22.0         # 20 mm from the pilot face + 2 mm pilot
    shaft_dcut_flat: float = 4.5       # D-cut flat-to-round width
    shaft_dcut_length: float = 18.0    # D-cut from the shaft tip inward
    bolt_pattern_square: float = 31.0  # centre-to-centre
    bolt_dia: float = 3.0              # M3
    bolt_hole_depth: float = 4.5       # threaded blind holes in the mounting face
    motor_bolt_total_length: float = 13.0   # 3 mm head + 10 mm thread
    motor_bolt_thread_length: float = 10.0
    motor_bolt_head_dia: float = 5.3
    motor_bolt_head_height: float = 3.0
    motor_bolt_thread_margin: float = 0.5   # [DESIGN] thread engagement kept short of the hole bottom
    motor_bolt_recess: float = 1.0     # [DESIGN] bolt-head top below the motor-plate inner face (stack-up)
    pilot_dia: float = 22.0            # NEMA 17 centring boss
    pilot_height: float = 2.0          # [DATASHEET] typical boss height (plate recess depth too)
    body_width: float = 42.3           # NEMA 17 standard
    body_length: float = 48.0


@dataclass(frozen=True)
class OutputHubParams:
    """Output hub / plate dimensions - spec section 5.3."""

    od: float = 70.3                   # 70 mm 6814 inner-race bore + 0.3 mm interference grip
    shaft_clearance_bore: float = 6.0  # 5 mm pin + 1 mm clearance
    output_hub_pin_ceiling: float = 1.0    # closed top above the blind pin holes
    proud_above_housing: float = 5.0   # output face past the housing output face (z=60 -> 65)
    arm_mount_bolt_circle_dia: float = 50.0
    arm_mount_bolt_count: int = 4      # 4x M4 clearance holes into captive nuts
    arm_mount_angle_offset_deg: float = 45.0   # offset from the output pins so the nut pockets clear them
    arm_mount_pocket_dia: float = 36.0     # central lightening recess in the proud face; 0 disables
    arm_mount_pocket_floor: float = 1.0    # solid floor left above the bearing-grip zone


@dataclass(frozen=True)
class PETGTolerances:
    """PETG fit adjustments - spec section 6 (mid-points of the spec ranges)."""

    bearing_seat_bore_add: float = 0.2     # 625 outer-race seat
    ring_pin_press_sub: float = -0.20      # subtractive: 4.0 - (-0.20) = 4.20 mm clearance holes
    d_bore_clearance_add: float = 0.065    # +0.13 mm diametral clearance for the motor-shaft D-bore
    dowel_bore_clearance_add: float = 0.075    # steel dowel in PETG
    mating_surface_add: float = 0.15
    bolt_clearance_add: float = 0.4        # [DESIGN] clearance added to every bolt / bolt-head diameter


@dataclass(frozen=True)
class ProfileParams:
    """Epitrochoid profile generation settings."""

    num_points: int = 2000     # points per revolution (periodic spline through all of them)


@dataclass(frozen=True)
class StackUp:
    """Axial stack-up - spec section 4. Z=0 is the outer face of the motor plate; +Z inward."""

    motor_plate_wall: float = 5.0
    motor_plate_inner_wall: float = 4.0    # inner wall thickness of the motor plate
    input_clearance: float = 4.0           # motor-plate inner face -> disc 1
    disc_thickness: float = 10.0
    inter_disc_spacer: float = 2.0
    output_clearance: float = 2.0          # disc 2 -> output bearings
    output_bearing_total: float = 20.0     # 2 x 6814 width
    output_wall: float = 3.0               # ring-gear-body output wall: retention lip + nut-pocket zone

    @property
    def z_motor_plate_inner(self) -> float:
        return self.motor_plate_wall + self.motor_plate_inner_wall      # 9

    @property
    def disc_zone(self) -> float:
        """Motor-plate inner face -> disc 2 outer face (26)."""
        return self.input_clearance + 2 * self.disc_thickness + self.inter_disc_spacer

    @property
    def bore_zone(self) -> float:
        """Disc zone + output clearance = the 116 mm bore length (28)."""
        return self.disc_zone + self.output_clearance

    @property
    def z_disc1(self) -> float:
        return self.z_motor_plate_inner + self.input_clearance          # 13

    @property
    def z_disc2(self) -> float:
        return self.z_disc1 + self.disc_thickness + self.inter_disc_spacer   # 25

    @property
    def z_output_bearings(self) -> float:
        return self.z_disc2 + self.disc_thickness + self.output_clearance    # 37

    @property
    def z_bearing_top(self) -> float:
        return self.z_output_bearings + self.output_bearing_total       # 57

    @property
    def total_housing_depth(self) -> float:
        return self.z_bearing_top + self.output_wall                    # 60

    @property
    def ring_gear_body_height(self) -> float:
        return self.total_housing_depth - self.z_motor_plate_inner      # 51


@dataclass(frozen=True)
class DriveConfig:
    """Top-level configuration aggregating all parameter groups."""

    gear: GearParams = field(default_factory=GearParams)
    disc: DiscParams = field(default_factory=DiscParams)
    shaft: ShaftParams = field(default_factory=ShaftParams)
    bearings: BearingParams = field(default_factory=BearingParams)
    housing: HousingParams = field(default_factory=HousingParams)
    motor: MotorParams = field(default_factory=MotorParams)
    output_hub: OutputHubParams = field(default_factory=OutputHubParams)
    tolerances: PETGTolerances = field(default_factory=PETGTolerances)
    profile: ProfileParams = field(default_factory=ProfileParams)
    stack_up: StackUp = field(default_factory=StackUp)


def compute_housing_bolt_angles(cfg: DriveConfig) -> list[float]:
    """Evenly spaced housing-bolt angles (radians, from +X). The 125 mm bolt circle and the
    108 mm ring-pin circle are ~8.5 mm apart radially, so no angular offset is needed (a bolt may
    line up with a ring pin: DEFAULT_CONFIG's do every 120 degrees)."""
    n = cfg.housing.bolt_count
    return [2 * math.pi * i / n for i in range(n)]


# The port: the CadQuery drive exactly (the designed parts' REFERENCE_BUILD - tests/cycloidal/test_port.py).
LEGACY_CONFIG = DriveConfig()

# What the parts build: the housing on 6 bolts, not 8 - 6 pillars at 60 degrees from +X (same start), so the
# j1_coupler yoke holds the two that straddle its bottom (lib/yaw_coupler/params.py DEFAULT).
DEFAULT_CONFIG = replace(LEGACY_CONFIG, housing=replace(LEGACY_CONFIG.housing, bolt_count=6))   # [DESIGN]
