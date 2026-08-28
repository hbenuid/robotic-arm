"""Central parameter definitions for the cycloidal drive.

Every dimension from the spec lives here. Part files import DriveConfig
and never hardcode numbers.
"""

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class GearParams:
    """Core gear geometry — spec Section 1.1 & 1.2."""

    num_lobes: int = 20
    num_ring_pins: int = 21  # N + 1
    eccentricity: float = 1.5  # mm
    ring_pin_circle_dia: float = 108.0  # mm
    ring_pin_dia: float = 4.0  # mm, h6 ground steel
    ring_pin_length: float = 35.0  # mm (4mm motor plate + 27mm bore zone + 4mm bearing-zone wall)

    @property
    def ring_pin_circle_radius(self) -> float:
        return self.ring_pin_circle_dia / 2.0

    @property
    def ring_pin_radius(self) -> float:
        return self.ring_pin_dia / 2.0

    @property
    def gear_ratio(self) -> int:
        return self.num_lobes

    @property
    def disc2_phase_deg(self) -> float:
        """Disc-2 epitrochoid phase offset relative to disc 1, in degrees.

        At orbit angle φ=π (disc 2's position, opposite disc 1's φ=0), the
        meshing kinematics require disc rotation = -π/N_lobes = -180°/N_lobes.
        Applied as a profile-only rotation in build_cycloidal_disc so the
        output-pin holes stay at standard disc-local angles and align with
        the stationary output pins after the (-e, 0) orbital translation.
        """
        return -180.0 / self.num_lobes


@dataclass(frozen=True)
class DiscParams:
    """Cycloidal disc dimensions — spec Section 1.2 & 1.3."""

    thickness: float = 10.0  # mm, matches 6003 bearing width
    center_bore_dia: float = 35.10  # mm, 35mm bearing OD + 0.10mm clearance
    inter_disc_spacer: float = 2.0  # mm
    output_pin_count: int = 4
    output_pin_circle_dia: float = 60.0  # mm
    output_pin_dia: float = 4.0  # mm (4mm × 45mm h6 ground steel dowel, captured in blind hub holes)
    output_pin_length: float = 45.0  # mm, h6 ground steel dowel
    output_pin_hole_dia: float = 7.4  # mm (4mm + 2*1.5mm ecc + 0.4mm clearance; 0.2mm radial slack ≈ ±0.38° output backlash — practical floor for 4 rigid pins)
    lobe_chamfer: float = 1.0  # mm, 45° chamfer on outer epitrochoid edge (assembly lead-in, hides elephant foot)


@dataclass(frozen=True)
class ShaftParams:
    """Eccentric shaft dimensions — spec Section 1.4."""

    bearing_seat_od: float = 17.10  # mm, slight clearance for 6003 bore
    eccentricity: float = 1.5  # mm
    spine_od: float = 5.0  # mm, shaft OD outside the lobe regions
    # Steel dowel pin replaces printed output stub for 625 bearing support
    support_pin_dia: float = 5.0  # mm, ground steel dowel pin h6
    support_pin_length: float = 20.0  # mm, ground steel dowel pin h6
    support_pin_hole_depth: float = 11.0  # mm, shortened by 1mm so D-bore (now 14mm) keeps a 1mm wall to the pin hole; pin protrudes 2mm proud instead of 1mm
    # Direct D-shaft engagement with motor shaft
    input_collar_od: float = 10.0  # mm, enlarged input section for D-bore wall
    d_bore_dia: float = 5.0  # mm, matches motor shaft (tolerance applied in builder)
    d_bore_flat: float = 4.5  # mm, D-flat width (matches motor shaft D-cut)
    d_bore_depth: float = 14.0  # mm, deepened to clear 22mm motor shaft past 9mm motor plate (13mm engagement + 1mm bottom clearance)


@dataclass(frozen=True)
class BearingParams:
    """All bearing dimensions — spec Section 2."""

    # 6003-2RS (eccentric bearings)
    ecc_bore: float = 17.0  # mm
    ecc_od: float = 35.0  # mm
    ecc_width: float = 10.0  # mm
    ecc_qty: int = 2

    # 6814-2RS (output bearings)
    out_bore: float = 70.0  # mm
    out_od: float = 90.0  # mm
    out_width: float = 10.0  # mm
    out_qty: int = 2

    # 625-2RS (eccentric shaft support, output side)
    inp_bore: float = 5.0  # mm
    inp_od: float = 16.0  # mm
    inp_width: float = 5.0  # mm
    inp_qty: int = 1


@dataclass(frozen=True)
class HousingParams:
    """Housing dimensions — spec Section 5."""

    od: float = 140.0  # mm (sized for 3mm+ wall around counterbores/nut pockets)
    bore_dia: float = 116.0  # mm
    wall_thickness: float = 12.0  # mm (140 - 116) / 2
    edge_chamfer: float = 1.5  # mm, 45° chamfer on the outer OD edges (both faces) of both housing parts; hides elephant foot, breaks sharp corners. 0 disables.
    motor_plate_wall: float = 5.0  # mm
    bolt_count: int = 8
    bolt_circle_dia: float = 125.0  # mm (outside 116mm bore)
    bolt_dia: float = 4.0  # mm (M4)
    bolt_length: float = 55.0  # mm (M4 × 55 SHCS)
    bolt_head_dia: float = 7.0  # mm (M4 SHCS head)
    bolt_head_height: float = 4.0  # mm (M4 SHCS head)
    bolt_counterbore_dia: float = 7.4  # mm (head 7mm + 0.4mm clearance)
    bolt_counterbore_depth: float = 4.5  # mm (4mm head + 0.5mm recess)
    bolt_nut_pocket_af: float = 7.2  # mm (M4 nut 7mm AF + 0.2mm pocket clearance)
    bolt_nut_thickness: float = 3.2  # mm (M4 hex nut)
    bolt_nut_depth: float = 4.0  # mm (pocket depth in ring gear body output face + hub arm-mount)
    output_bearing_seat_dia: float = 90.15  # mm (press fit for 6814 outer race)


@dataclass(frozen=True)
class MotorParams:
    """NEMA 17 motor dimensions — spec Section 3.1."""

    shaft_dia: float = 5.0  # mm
    shaft_length: float = 22.0  # mm (20mm from pilot face + 2mm pilot)
    shaft_dcut_flat: float = 4.5  # mm, D-cut flat-to-round width
    shaft_dcut_length: float = 18.0  # mm, D-cut extends from shaft tip inward
    bolt_pattern_square: float = 31.0  # mm (center-to-center)
    bolt_dia: float = 3.0  # mm (M3)
    bolt_hole_depth: float = 4.5  # mm, threaded blind holes in mounting face
    motor_bolt_total_length: float = 13.0  # mm (3mm head + 10mm thread)
    motor_bolt_thread_length: float = 10.0  # mm
    motor_bolt_head_dia: float = 5.3  # mm
    motor_bolt_head_height: float = 3.0  # mm
    pilot_dia: float = 22.0  # mm (NEMA 17 centering boss)
    body_width: float = 42.3  # mm (NEMA 17 standard)
    body_length: float = 48.0  # mm


@dataclass(frozen=True)
class OutputHubParams:
    """Output hub/plate dimensions — spec Section 5.3."""

    od: float = 70.3  # mm, 70mm 6814 inner-race bore + 0.3mm interference grip
    shaft_clearance_bore: float = 6.0  # mm (5mm pin + 1mm clearance)
    output_hub_pin_ceiling: float = 1.0  # mm, closed-top thickness above blind pin holes

    # Arm-link interface: output face protrudes past the chassis so the arm link
    # clears the stationary housing (housing output face is at total_housing_depth).
    proud_above_housing: float = 5.0  # mm, output face protrudes this far past the housing output face (z=60); keeps a 5mm air gap clear of the housing
    arm_mount_bolt_circle_dia: float = 50.0  # mm, arm-link bolt circle (r=25, clears center + output pins)
    arm_mount_bolt_count: int = 4  # 4× M4 clearance holes (bolts thread into captive nuts)
    arm_mount_angle_offset_deg: float = 45.0  # offset from output pins (0/90/180/270) so nut pockets clear them
    # Central lightening recess in the proud arm-mount face — reclaims dead material
    # from the solid block above the bearing-grip zone.  r=18mm leaves ~4.8mm wall to
    # the arm-bolt edges (r=22.8mm) and clears the pin-hole columns (r=27.9mm).
    arm_mount_pocket_dia: float = 36.0  # mm; set 0.0 to disable (keep a fully sealed face)
    arm_mount_pocket_floor: float = 1.0  # mm solid floor left above the bearing-grip zone



@dataclass(frozen=True)
class PETGTolerances:
    """PETG-specific fit adjustments — spec Section 6.

    Applied to nominal dimensions in part builders. Midpoints of spec ranges.
    """

    bearing_seat_bore_add: float = 0.2  # 625 outer-race seat offset; prints to ~16.0–16.1mm (slip-to-light press)
    bearing_inner_shaft_sub: float = 0.075  # -0.05 to -0.10mm
    ring_pin_press_sub: float = -0.20  # subtractive: 4.0 - (-0.20) = 4.20mm clearance holes
    sliding_clearance_add: float = 0.25  # +0.20 to +0.30mm
    d_bore_clearance_add: float = 0.065  # +0.13mm diametral clearance for motor shaft D-bore
    dowel_bore_clearance_add: float = 0.075  # clearance fit for steel dowel in PETG
    mating_surface_add: float = 0.15  # +0.15mm


@dataclass(frozen=True)
class ProfileParams:
    """Epitrochoid profile generation settings."""

    num_points: int = 2000  # points per revolution
    spline_tolerance: float = 0.001  # mm, for CadQuery makeSpline


@dataclass(frozen=True)
class StackUp:
    """Axial stack-up Z positions — spec Section 4.

    Z=0 is the outer face of the motor plate (motor mounting face).
    All values in mm, measured from Z=0 inward.
    """

    motor_plate_wall: float = 5.0
    motor_plate_inner_wall: float = 4.0  # inner wall thickness of motor plate
    input_clearance: float = 4.0  # gap between motor plate inner face and disc 1
    disc_thickness: float = 10.0
    inter_disc_spacer: float = 2.0
    output_clearance: float = 2.0  # gap between disc 2 and output bearings
    output_bearing_total: float = 20.0  # 2 x 6814 width
    output_wall: float = 3.0  # ring gear body output-end wall: bearing-retention lip + captive nut-pocket zone. M4×55 bolt (counterbore 4.5 + 55 = 59.5mm) lands inside the 60mm depth with full nut engagement

    @property
    def z_motor_plate_inner(self) -> float:
        return self.motor_plate_wall + self.motor_plate_inner_wall  # 9mm

    @property
    def disc_zone(self) -> float:
        """Axial span from motor plate inner face to disc 2 outer face (26mm)."""
        return self.input_clearance + 2 * self.disc_thickness + self.inter_disc_spacer

    @property
    def z_disc1(self) -> float:
        return self.z_motor_plate_inner + self.input_clearance  # 13mm (9 + 4)

    @property
    def z_disc2(self) -> float:
        return self.z_disc1 + self.disc_thickness + self.inter_disc_spacer  # 25mm

    @property
    def z_output_bearings(self) -> float:
        return self.z_disc2 + self.disc_thickness + self.output_clearance  # 37mm

    @property
    def z_bearing_top(self) -> float:
        return self.z_output_bearings + self.output_bearing_total  # 57mm (6814 stack top / retention-lip start)

    @property
    def total_housing_depth(self) -> float:
        return self.z_bearing_top + self.output_wall  # 60mm


@dataclass
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


def compute_housing_bolt_angles(cfg: "DriveConfig") -> list[float]:
    """Compute evenly-spaced bolt angles for housing bolts.

    Bolts are placed at uniform angular intervals. The bolt circle
    (125mm dia) and ring pin circle (108mm dia) have sufficient radial
    separation (~8.5mm) that angular alignment with pins does not cause
    interference.

    Returns a sorted list of bolt angles in radians.
    """
    n_bolts = cfg.housing.bolt_count
    return [2 * math.pi * i / n_bolts for i in range(n_bolts)]


# Default configuration instance
DEFAULT_CONFIG = DriveConfig()
