"""Export all custom parts to STL and STEP files, plus a full assembly STEP.

Run:  python export.py
Output goes to ./export/stl/ and ./export/step/
"""

import os

import cadquery as cq

from src.params import DEFAULT_CONFIG
from src.cycloidal_disc import build_cycloidal_disc
from src.eccentric_shaft import build_eccentric_shaft
from src.motor_plate import build_motor_plate
from src.ring_gear_body import build_ring_gear_body
from src.output_hub import build_output_hub
from src.purchased_parts import (
    build_bearing_6003,
    build_bearing_6814,
    build_bearing_625,
    build_nema17_motor,
    build_ring_pins,
    build_output_pins,
    build_shaft_support_pin,
)

ROOT = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(ROOT, "export", "stl")
STEP_DIR = os.path.join(ROOT, "export", "step")


def export_part(part: cq.Workplane, name: str) -> None:
    stl_path = os.path.join(STL_DIR, f"{name}.stl")
    step_path = os.path.join(STEP_DIR, f"{name}.step")
    cq.exporters.export(part, stl_path)
    cq.exporters.export(part, step_path)
    print(f"  {name}.stl + {name}.step")


def build_assembly() -> cq.Assembly:
    """Build the full drive assembly with all parts positioned."""
    cfg = DEFAULT_CONFIG
    stack = cfg.stack_up
    e = cfg.gear.eccentricity

    assy = cq.Assembly()

    # Disc 1
    disc1 = build_cycloidal_disc()
    assy.add(disc1, name="disc_1", loc=cq.Location((e, 0, stack.z_disc1)))

    # Disc 2 (phase-shifted profile, no assembly rotation)
    disc2 = build_cycloidal_disc(phase_offset_deg=cfg.gear.disc2_phase_deg)
    assy.add(disc2, name="disc_2", loc=cq.Location((-e, 0, stack.z_disc2)))

    # 6003 eccentric bearings
    assy.add(build_bearing_6003(), name="bearing_6003_1",
             loc=cq.Location((e, 0, stack.z_disc1)))
    assy.add(build_bearing_6003(), name="bearing_6003_2",
             loc=cq.Location((-e, 0, stack.z_disc2)))

    # 6814 output bearings
    assy.add(build_bearing_6814(), name="bearing_6814_1",
             loc=cq.Location((0, 0, stack.z_output_bearings)))
    assy.add(build_bearing_6814(), name="bearing_6814_2",
             loc=cq.Location((0, 0, stack.z_output_bearings + cfg.bearings.out_width)))

    # Eccentric shaft (already positioned by its builder)
    assy.add(build_eccentric_shaft(), name="eccentric_shaft")

    # Ring pins
    pin_engagement = (cfg.gear.ring_pin_length - stack.disc_zone) / 2.0
    z_pins = stack.z_motor_plate_inner - pin_engagement
    assy.add(build_ring_pins(), name="ring_pins",
             loc=cq.Location((0, 0, z_pins)))

    # Output pins
    assy.add(build_output_pins(), name="output_pins",
             loc=cq.Location((0, 0, stack.z_disc1)))

    # Motor
    assy.add(build_nema17_motor(), name="nema17_motor")

    # Motor plate
    assy.add(build_motor_plate(), name="motor_plate")

    # Ring gear body
    assy.add(build_ring_gear_body(), name="ring_gear_body",
             loc=cq.Location((0, 0, stack.z_motor_plate_inner)))

    # Output hub
    assy.add(build_output_hub(), name="output_hub",
             loc=cq.Location((0, 0, stack.z_output_bearings)))

    # Output shaft pin (steel dowel)
    z_pin_base = stack.z_disc2 + cfg.disc.thickness - cfg.shaft.support_pin_hole_depth
    assy.add(build_shaft_support_pin(), name="shaft_support_pin",
             loc=cq.Location((0, 0, z_pin_base)))

    # 625 bearing
    assy.add(build_bearing_625(), name="bearing_625",
             loc=cq.Location((0, 0, stack.z_output_bearings)))

    return assy


def main() -> None:
    os.makedirs(STL_DIR, exist_ok=True)
    os.makedirs(STEP_DIR, exist_ok=True)

    # Individual parts
    parts = {
        "cycloidal_disc_1": build_cycloidal_disc(),
        "cycloidal_disc_2": build_cycloidal_disc(
            phase_offset_deg=DEFAULT_CONFIG.gear.disc2_phase_deg
        ),
        "eccentric_shaft": build_eccentric_shaft(),
        "motor_plate": build_motor_plate(),
        "ring_gear_body": build_ring_gear_body(),
        "output_hub": build_output_hub(),
    }

    print(f"Exporting {len(parts)} parts to STL and STEP...")
    for name, part in parts.items():
        export_part(part, name)

    # Full assembly STEP
    print("  Building full assembly...")
    assy = build_assembly()
    assy_path = os.path.join(STEP_DIR, "cycloidal_drive_assembly.step")
    assy.export(assy_path)
    print("  cycloidal_drive_assembly.step")

    print("Done.")


if __name__ == "__main__":
    main()
