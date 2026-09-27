"""forearm_roll_drive - the belt-driven forearm roll (the 6th joint), a code-driven module like the cycloidal drive.

Stator: the elbow block - ONE printed part that is the elbow's output flange (its underside repeats the SolidWorks
j3_coupler's lip, boss, journal and stub down into j1_link's bore; the elbow 90T pulley bolts straight into it, so
j3_coupler#1 is retired, lib/placements.py RETIRED) and the roll housing round the shaft that crosses the elbow axis
(bearing 1's seat in its rear end, the Ø62 cavity the ring runs in, open to the front face) -, bearing 1, the bolt-on
end cap with bearing 2 on the front face, the bolt-on motor mount (its base in the step on the block's top, 4x M3
countersunk into M3 nuts in the block's channels), the 40 mm kit motor + MKS board on the mount's vertical plate UP in
the swing plane (its body behind the elbow axis) and the 20T on the motor shaft. Rotor: the hollow
roll shaft with its integral 90T ring between the bearings and the end spigot the forearm's wall bolts onto
(forearm_link). MODULE FRAME (lib/forearm/params.py RollDriveParams): origin on the roll axis at the elbow-axis
crossing, +Z along the roll axis toward the wrist, +X = host +Z (N), +Y = up in the arm's swing plane; the arm places
it through lib/mounts.py MODULE_MOUNTS ("forearm_roll_drive#1" on j2_link#1). Every station comes from
lib/forearm stack_positions() - never type one here. Gear ratio 90 / 20 = 4.5 (FOREARM_ROLL_RATIO); the belt
(240-2GT) and the self-tapping / motor M3 screws are tools/bom.py EXTRAS, the mount's countersunk M3s + nuts are rows
here; the elbow 90T's M4 screws + nuts into the block are the arm's (lib/mounts.py FASTENER_MOUNTS).
"""
from cadgen import step

from assemblies._occurrences import located_children
from lib import reference as R
from lib.assembly import assembly
from lib.forearm import DEFAULT, stack_positions
from lib.models import raw

S = stack_positions(DEFAULT)

# The module's printed parts' colour (assemblies/arm.py MODULE_TINTS reuses it); purchased parts are BOUGHT_TINT.
TINT = "#2A9D8F"


def _at(x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (x, y, z)


# (part, role | None, placement in the module frame) - a position, or ((x, y, z), (rx, ry, rz)) when the part's own
# frame is not the module's (the 20T's vendor axis is +X). The block, the motor mount and the shaft are built at their
# stations (their rows are at 0), the cap at its local origin (placed on the block's front face); the bearings stand on
# their seats; the mount's screws and nuts are patterns on Z turned down -Y (Rot X +90: their (x, y) = the module's
# (x, z)), the heads flush with the block's top, the nuts up against their channels' ceilings; the motor's mounting
# face is the mount plate's rear face, centred on the roll axis in X, spun so its connector points +X.
OCCURRENCES = [
    ("forearm_roll_block",    None, _at()),
    ("forearm_roll_motor_mount", None, _at()),
    ("forearm_roll_mount_screws", None, ((0.0, DEFAULT.drive.block_y[1], 0.0), (90.0, 0.0, 0.0))),
    ("forearm_roll_mount_nuts",   None, ((0.0, S["y_mount_nut"], 0.0), (90.0, 0.0, 0.0))),
    ("bearing_6808",          "1",  _at(z=S["z_bearing_1"])),
    ("forearm_roll_shaft",    None, _at()),
    ("bearing_6808",          "2",  _at(z=S["z_bearing_2"])),
    ("forearm_roll_retainer", None, _at(z=S["z_cap"])),                                             # the end cap on the block's front face
    # the roll motor, its board and its 20T - up in the swing plane (+Y) - carry the joint as their role (labels unique in the arm)
    ("nema17_40mm",           "forearm_roll", ((S["x_motor"], S["y_motor"], S["z_motor_face"]), (0.0, 0.0, DEFAULT.drive.motor_spin_deg))),    # spun: connector toward +X
    ("mks_servo42d",          "forearm_roll", ((S["x_motor"], S["y_motor"], S["z_motor_board"]), (0.0, 0.0, DEFAULT.drive.motor_spin_deg))),
    ("gt2_pulley_20t",        "forearm_roll", ((S["x_motor"], S["y_motor"], S["z_20t"]), (0.0, -90.0, 0.0))),   # hub face pulley_lift above the plate, bore axis along +Z
]

# The module's rigid bodies (robot/frames.py LINKS: "forearm_roll_drive#1:stator" in elbow_link, ":rotor" in forearm_link).
ROTOR = frozenset({"forearm_roll_shaft"})
BODIES = {"rotor": ROTOR, "stator": frozenset(part for part, _, _ in OCCURRENCES) - ROTOR}

# Totals lock (tests/test_assembly.py, test_robot.py; robot/ inertials sum the same rows): whole module + per body.
# Re-derive with totals() / totals("stator") after any geometry change.
EXPECTED = {
    "leaves": 11, "solids": 32, "solid_volume": 419018.261,
    "bodies": {"stator": {"leaves": 10, "solids": 31, "solid_volume": 323775.82},
               "rotor": {"leaves": 1, "solids": 1, "solid_volume": 95242.441}},
}


@step
def forearm_roll_drive():
    """The drive in its module frame (printed parts TINT, purchased parts BOUGHT_TINT)."""
    return assembly("forearm_roll_drive", located_children(OCCURRENCES, tint=TINT))


def totals(body: str | None = None) -> dict:
    """(leaves, solids, solid_volume) of a fresh in-process build - the whole module or one of BODIES."""
    module = raw(forearm_roll_drive)
    leaves = [c for c in module.children if body is None or c.label.split(":")[0] in BODIES[body]]
    return {"leaves": len(leaves), "solids": sum(len(c.solids()) for c in leaves),
            "solid_volume": round(sum(R.solid_volume(c) for c in leaves), 3)}
