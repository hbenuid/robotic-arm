"""forearm_roll_drive - the belt-driven forearm roll (the 6th joint), a code-driven module like the cycloidal drive.

Stator: the elbow block (bolted to j3_coupler#1 through the SolidWorks disc's interface, riding with the elbow
pulley in elbow_link), both 6808 bearings in its one seat, the bolted retainer, the 40 mm kit motor + MKS board on
the block's pad tower and the 20T on the motor shaft. Rotor: the hollow roll shaft with its integral 90T ring and
the flange that bolts to the forearm's wall (forearm_link). MODULE FRAME (lib/forearm/params.py RollDriveParams):
origin on the roll axis at the elbow-axis crossing, +Z along the roll axis toward the wrist, +X = the motor side (N);
the arm places it through lib/mounts.py MODULE_MOUNTS ("forearm_roll_drive#1" on j2_link#1). Every station comes
from lib/forearm stack_positions() - never type one here. Gear ratio 90 / 20 = 4.5 (FOREARM_ROLL_RATIO); the
belt (230-2GT), the fasteners and the stop pin are tools/bom.py EXTRAS.
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
# frame is not the module's (the 20T's vendor axis is +X). The block, the shaft and the retainer are built at their
# stations (their rows are at 0); the bearings stand on their seats; the motor's mounting face is the pad's top.
OCCURRENCES = [
    ("forearm_roll_block",    None, _at()),
    ("bearing_6808",          "1",  _at(z=S["z_bearing_1"])),
    ("forearm_roll_shaft",    None, _at()),
    ("bearing_6808",          "2",  _at(z=S["z_bearing_2"])),
    ("forearm_roll_retainer", None, _at(z=S["z_retainer"])),
    # the roll motor, its board and its 20T carry the joint as their role (labels unique in the arm, like the mounted motors')
    ("nema17_40mm",           "forearm_roll", _at(x=S["x_motor"], z=S["z_motor_face"])),
    ("mks_servo42d",          "forearm_roll", _at(x=S["x_motor"], z=S["z_motor_board"])),
    ("gt2_pulley_20t",        "forearm_roll", ((S["x_motor"], 0.0, S["z_20t"]), (0.0, -90.0, 0.0))),   # hub face pulley_lift above the pad, bore axis along +Z
]

# The module's rigid bodies (robot/frames.py LINKS: "forearm_roll_drive#1:stator" in elbow_link, ":rotor" in forearm_link).
ROTOR = frozenset({"forearm_roll_shaft"})
BODIES = {"rotor": ROTOR, "stator": frozenset(part for part, _, _ in OCCURRENCES) - ROTOR}

# Totals lock (tests/test_assembly.py, test_robot.py; robot/ inertials sum the same rows): whole module + per body.
# Re-derive with totals() / totals("stator") after any geometry change.
EXPECTED = {
    "leaves": 8, "solids": 23, "solid_volume": 257505.936,
    "bodies": {"stator": {"leaves": 7, "solids": 22, "solid_volume": 202218.409},
               "rotor": {"leaves": 1, "solids": 1, "solid_volume": 55287.527}},
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
