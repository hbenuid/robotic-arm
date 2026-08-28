"""The 20:1 cycloidal drive (the shoulder-pitch actuator) as a code-driven module: 18 parts
placed from lib/cycloidal stack_positions, i.e. the drive repo's assembly.py layout (ring pins
z 5.5, output pins z 11 - not its export.py's), fasteners included.

Frame: the drive repo's frame - Z is the motor axis, z=0 the motor-plate OUTER face (NEMA 17
mounting face), the motor body in -Z, the housing z 0..60 and the output hub's arm-mount face at
z=65. assemblies/arm.py places the module at placements.json "cycloidal_drive#1" (the SolidWorks
node's pose: horizontal axis, housing in the j1_coupler yoke, hub face bolted to j1_link).

Run:  ./cadtool step assemblies/cycloidal_drive.py            -> assemblies/cycloidal_drive.step (git-ignored)
      ./cadtool python -m assemblies.cycloidal_drive          -> preview in the OCP CAD Viewer
      ./cadtool python -m assemblies.cycloidal_drive --totals -> leaves / solids / volume / bbox (the EXPECTED lock)
"""
# --- path shim: files inside assemblies/ -> parent.parent (= cad/) --------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import Location  # noqa: E402

from assemblies._occurrences import add_located  # noqa: E402
from lib.assembly import AssemblyHelper  # noqa: E402
from lib.cycloidal import DEFAULT_CONFIG, stack_positions  # noqa: E402

S = stack_positions(DEFAULT_CONFIG)


def _at(x: float = 0.0, z: float = 0.0) -> Location:
    return Location((x, 0.0, z))


# (part, role, Location in the module frame) in the drive repo's assembly.py order. Every number
# comes from stack_positions() - never type one here.
OCCURRENCES = [
    ("cycloidal_disc_1",           None, _at(S["x_disc1"], S["z_disc1"])),
    ("bearing_6003",               "1",  _at(S["x_disc1"], S["z_disc1"])),
    ("cycloidal_disc_2",           None, _at(S["x_disc2"], S["z_disc2"])),
    ("bearing_6003",               "2",  _at(S["x_disc2"], S["z_disc2"])),
    ("bearing_6814",               "1",  _at(z=S["z_6814_1"])),
    ("bearing_6814",               "2",  _at(z=S["z_6814_2"])),
    ("cycloidal_eccentric_shaft",  None, _at(z=S["z_eccentric_shaft"])),   # built at its stack position
    ("cycloidal_ring_pins",        None, _at(z=S["z_ring_pins"])),
    ("cycloidal_output_pins",      None, _at(z=S["z_output_pins"])),
    ("nema17_48mm",                None, _at(z=S["z_motor"])),             # mounting face at z=0
    ("cycloidal_motor_bolts",      None, _at(z=S["z_motor_bolts"])),
    ("cycloidal_motor_plate",      None, _at(z=S["z_motor_plate"])),
    ("cycloidal_ring_gear_body",   None, _at(z=S["z_ring_gear_body"])),
    ("cycloidal_output_hub",       None, _at(z=S["z_hub"])),
    ("cycloidal_shaft_support_pin", None, _at(z=S["z_support_pin"])),
    ("bearing_625",                None, _at(z=S["z_625"])),
    ("cycloidal_housing_bolts",    None, _at(z=S["z_housing_bolts"])),
    ("cycloidal_housing_nuts",     None, _at(z=S["z_housing_nuts"])),
]

# Totals of gen_step() (tests/test_cycloidal_assembly.py locks them; refresh with --totals after a
# geometry change): 18 leaves, 38 SolidWorks-equivalent solids + 20 fasteners.
EXPECTED = {"leaves": 18, "solids": 58, "solid_volume": 691936.788}


def gen_step():
    """The drive as a labelled Compound 'cycloidal_drive' in the module frame."""
    asm = AssemblyHelper("cycloidal_drive")
    add_located(asm, OCCURRENCES)
    return asm.build()


def totals():
    from lib import reference as R

    shape = gen_step()
    leaves = [n for n in shape.children]
    bb = shape.bounding_box()
    return {
        "leaves": len(leaves), "solids": len(shape.solids()), "solid_volume": round(R.solid_volume(shape), 3),
        "bbox_min": [round(v, 3) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
        "bbox_size": [round(v, 3) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
    }


if __name__ == "__main__":
    if "--totals" in sys.argv:
        print(totals())
    else:
        from ocp_vscode import show
        show(gen_step())
