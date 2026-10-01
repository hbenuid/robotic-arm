"""The 21:1 cycloidal drive (the shoulder-pitch actuator) as a code-driven module: its parts placed from lib/cycloidal
stack_positions, i.e. the drive repo's assembly.py layout for the gear stack (the discs, the shaft, the motor) with
the turning shell's (lib/cycloidal/params.py ShellParams: one 6814 at each end, the shell the same at both),
fasteners included, plus the MKS SERVO42D board kit on the motor's rear face (parts/joints/mks_servo42d,
"z_mks_board"). The shell's body round the discs is not here: it is printed as one with j1_link (the upper arm rises
off it - lib/upper_arm/link.py, placed by assemblies/arm.py), which this module's shell ring bolts to.

Frame: the drive repo's frame - Z is the motor axis, z=0 the motor plate's OUTER face (NEMA 17 mounting face), the
motor body in -Z (its board to z=-62.1), the shell z -13..61 (the shell ring -13..9, the body 9..61), the middle of
the discs at z=24, the hub's face at z=62 and the sleeve's end at z=-22 on the j1_coupler yoke's two legs.
assemblies/arm.py places the module at placements.json "cycloidal_drive#1" (the SolidWorks node's pose: horizontal
axis).

Run:  ./cadtool gen assemblies/cycloidal_drive.py             -> assemblies/cycloidal_drive.step (git-ignored)
      ./cadtool viewer                                        -> http://127.0.0.1:3245/?file=assemblies/cycloidal_drive.step
      ./cadtool python -c "from assemblies.cycloidal_drive import totals; print(totals(), totals('rotor'))"
                                                              -> leaves / solids / volume / bbox (the EXPECTED lock, whole or per body)
"""

from __future__ import annotations

from cadgen import step

from assemblies._occurrences import located_children
from lib.assembly import assembly
from lib.cycloidal import DEFAULT_CONFIG, stack_positions
from lib.models import raw

S = stack_positions(DEFAULT_CONFIG)

# The color of the drive's PRINTED parts, here and in the arm (arm.py MODULE_TINTS); its purchased
# parts are _occurrences.BOUGHT_TINT grey in both.
TINT = "#C44E52"


def _at(x: float = 0.0, z: float = 0.0) -> tuple[float, float, float]:
    return (x, 0.0, z)


# (part, role, position in the module frame - mm, translations only: data, so importing this module
# never touches the kernel) in the drive repo's assembly.py order. Every number comes from
# stack_positions() - never type one here.
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
    ("mks_servo42d",               None, _at(z=S["z_mks_board"])),         # the kit's board on the motor's rear face (z=-48)
    ("cycloidal_motor_bolts",      None, _at(z=S["z_motor_bolts"])),
    ("cycloidal_motor_plate",      None, _at(z=S["z_motor_plate"])),
    ("cycloidal_shell_ring",       None, _at(z=S["z_shell_ring"])),       # built at its stack position
    ("cycloidal_output_hub",       None, _at(z=S["z_hub"])),
    ("cycloidal_shaft_support_pin", None, _at(z=S["z_support_pin"])),
    ("bearing_625",                None, _at(z=S["z_625"])),
    ("cycloidal_housing_bolts",    None, _at(z=S["z_housing_bolts"])),
    ("cycloidal_housing_nuts",     None, _at(z=S["z_housing_nuts"])),
]

# Rigid bodies of the drive for the robot description (robot/frames.py LINKS keys
# "cycloidal_drive#1:stator" / "cycloidal_drive#1:rotor", expanded by
# assemblies/_occurrences.world_rows): the drive IS the shoulder_pitch joint. rotor = what turns with the shell (its
# body is j1_link's): the shell ring, the ring pins, the housing bolts + nuts, and both 6814s (their outer races turn -
# a bearing rides with its housing). stator = everything the yoke holds: the motor plate, the output hub,
# its 4 output pins and the 625 in its pocket, the NEMA 17 + its bolts + its MKS board, and the gear train that moves
# about the axis at intermediate speeds (eccentric shaft, support pin, discs, 6003s) - lumped with the carrier, the
# usual URDF-inertial convention (axisymmetric about the joint axis).
ROTOR = frozenset({"cycloidal_shell_ring", "cycloidal_ring_pins", "cycloidal_housing_bolts", "cycloidal_housing_nuts",
                   "bearing_6814"})
BODIES = {"rotor": ROTOR, "stator": frozenset(part for part, _, _ in OCCURRENCES) - ROTOR}

# Totals of the model (tests/cycloidal/test_assembly.py locks them; refresh with totals() /
# totals(body) after a geometry change): the drive's own leaves and solids + the 7-solid vendor motor + 16 fasteners +
# the 13-solid board kit, and the same per rigid body.
EXPECTED = {
    "leaves": 19, "solids": 73, "solid_volume": 551156.383,
    "bodies": {
        "stator": {"leaves": 13, "solids": 37, "solid_volume": 407955.132},
        "rotor": {"leaves": 6, "solids": 36, "solid_volume": 143201.251},
    },
}


@step
def cycloidal_drive():
    """The drive as a labelled Compound 'cycloidal_drive' in the module frame: printed parts TINT,
    purchased parts BOUGHT_TINT grey."""
    return assembly("cycloidal_drive", located_children(OCCURRENCES, tint=TINT))


def totals(body: str | None = None, *, shape=None):
    """leaves / solids / volume / bbox of the module (in-process, or `shape`: a build of it already made), or of one
    rigid body (`body` in BODIES: the leaves whose part name - the label before ':role' - is in it)."""
    from lib import reference as R

    if shape is None:
        shape = raw(cycloidal_drive)   # the model BODY, in-process - never the model (that builds)
    leaves = [n for n in shape.children if body is None or n.label.split(":")[0] in BODIES[body]]
    boxes = [n.bounding_box() for n in leaves]
    lo = [min(getattr(b.min, ax) for b in boxes) for ax in "XYZ"]
    hi = [max(getattr(b.max, ax) for b in boxes) for ax in "XYZ"]
    return {
        "leaves": len(leaves),
        "solids": sum(len(n.solids()) for n in leaves),
        "solid_volume": round(sum(R.solid_volume(n) for n in leaves), 3),
        "bbox_min": [round(v, 3) for v in lo],
        "bbox_size": [round(h - l, 3) for h, l in zip(hi, lo, strict=True)],
    }


if __name__ == "__main__":
    cycloidal_drive()   # build: writes the sibling cycloidal_drive.step
