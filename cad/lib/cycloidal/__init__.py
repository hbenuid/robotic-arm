"""The 20:1 cycloidal drive, ported from the cycloidal_drive repo (docs/cycloidal_drive.md).

Pure-Python config and layout here; build123d builders in lib/cycloidal/disc.py and
lib/cycloidal/housing.py (imported explicitly by the parts that need them, so importing this
package never pulls in OCCT). lib/ never imports parts/.
"""
from lib.cycloidal.layout import (  # noqa: F401
    PILLAR_OVERSHOOT, arm_mount_angles, arm_mount_points, compute_housing_bolt_angles, hex_circumdiameter,
    housing_bolt_points, hub_height, motor_bolt_counterbore_depth, motor_bolt_points,
    output_pin_points, pillar_corners, pillar_half_width, ring_pin_engagement, ring_pin_hole_depth, ring_pin_hole_dia,
    ring_pin_points, stack_positions,
)
from lib.cycloidal.params import (  # noqa: F401
    DEFAULT_CONFIG, LEGACY_CONFIG, BearingParams, DiscParams, DriveConfig, GearParams, HousingParams, MotorParams,
    OutputHubParams, PETGTolerances, ProfileParams, ShaftParams, StackUp,
)
