"""forearm_roll_drive - the belt-driven forearm roll (the 6th joint), a code-driven module like the cycloidal drive.

Stator: the frame - ONE printed part (forearm_roll_block) round the roll motor: round about the elbow axis (the upper arm's
round end) and tangent up to the tower round the roll axis; its underside is the elbow's output flange (it repeats the
SolidWorks j3_coupler's lip, boss, journal and stub down into j1_link's bore; the elbow 90T pulley bolts straight into it,
so j3_coupler#1 is retired, lib/placements.py RETIRED); the 40 mm kit motor + MKS board ON the elbow axis in its pocket
(open on the +N face, the pocket's front wall the plate with the tension slots) and the 20T on the motor shaft, out in
front of the frame's front face; the 6806 pair back to back on the tower's lip. Rotor: the pulley (forearm_roll_pulley,
the output: the integral 90T ring sunk in the cup on the frame's front face, its hub down through bearing 2, the spigot
the forearm's wall bolts onto - 4x M3 into nuts in its core; forearm_link) and the shaft (forearm_roll_shaft, a collar:
its hub up through bearing 1 to the pulley's, the rotor clamp's nuts and the stop lug in its flange), clamped by 4x M3
from the pulley's front face. MODULE FRAME (lib/forearm/params.py RollDriveParams): origin on the roll axis at the elbow
axis' station, +Z along the roll axis toward the wrist, +X = host +Z (N), +Y = up in the arm's swing plane, the elbow
axis along X at y_elbow (the arm's elbow offset below the roll axis); the arm places it through lib/mounts.py
MODULE_MOUNTS ("forearm_roll_drive#1" on j2_link#1). Every station comes from lib/forearm stack_positions() - never type
one here. Gear ratio 90 / 20 = 4.5 (FOREARM_ROLL_RATIO); the belt (240-2GT), the rotor clamp's and the forearm wall's M3
screws + nuts and the motor's M3 are tools/bom.py EXTRAS; the elbow 90T's M4 screws + nuts into the frame are the arm's
(lib/mounts.py FASTENER_MOUNTS).
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
# frame is not the module's (the 20T's vendor axis is +X). The frame, the shaft and the pulley are built at their
# stations (their rows are at 0); the bearings stand on their seats; the motor's mounting face is the plate's rear face,
# the motor on the elbow axis (centred on the roll axis in X), spun so its connector points +X.
OCCURRENCES = [
    ("forearm_roll_block",    None, _at()),
    ("bearing_6806",          "1",  _at(z=S["z_bearing_1"])),
    ("forearm_roll_shaft",    None, _at()),
    ("bearing_6806",          "2",  _at(z=S["z_bearing_2"])),
    ("forearm_roll_pulley",   None, _at()),
    # the roll motor, its board and its 20T - on the elbow axis (-Y) - carry the joint as their role (labels unique in the arm)
    ("nema17_40mm",           "forearm_roll", ((S["x_motor"], S["y_motor"], S["z_motor_face"]), (0.0, 0.0, DEFAULT.drive.motor_spin_deg))),    # spun: connector toward +X
    ("mks_servo42d",          "forearm_roll", ((S["x_motor"], S["y_motor"], S["z_motor_board"]), (0.0, 0.0, DEFAULT.drive.motor_spin_deg))),
    ("gt2_pulley_20t",        "forearm_roll", ((S["x_motor"], S["y_motor"], S["z_20t"]), (0.0, -90.0, 0.0))),   # hub face pulley_lift in front of the plate, bore axis along +Z
]

# The module's rigid bodies (robot/frames.py LINKS: "forearm_roll_drive#1:stator" in elbow_link, ":rotor" in forearm_link).
ROTOR = frozenset({"forearm_roll_shaft", "forearm_roll_pulley"})
BODIES = {"rotor": ROTOR, "stator": frozenset(part for part, _, _ in OCCURRENCES) - ROTOR}

# Totals lock (tests/test_assembly.py, test_robot.py; robot/ inertials sum the same rows): whole module + per body.
# Re-derive with totals() / totals("stator") after any geometry change.
EXPECTED = {
    "leaves": 8, "solids": 23, "solid_volume": 429577.799,
    "bodies": {"stator": {"leaves": 6, "solids": 21, "solid_volume": 391317.151},
               "rotor": {"leaves": 2, "solids": 2, "solid_volume": 38260.648}},
}


@step
def forearm_roll_drive():
    """The drive in its module frame (printed parts TINT, purchased parts BOUGHT_TINT)."""
    return assembly("forearm_roll_drive", located_children(OCCURRENCES, tint=TINT))


def totals(body: str | None = None, *, shape=None) -> dict:
    """(leaves, solids, solid_volume) of a fresh in-process build - or of `shape`, a build of it already made - the
    whole module or one of BODIES."""
    module = raw(forearm_roll_drive) if shape is None else shape
    leaves = [c for c in module.children if body is None or c.label.split(":")[0] in BODIES[body]]
    return {"leaves": len(leaves), "solids": sum(len(c.solids()) for c in leaves),
            "solid_volume": round(sum(R.solid_volume(c) for c in leaves), 3)}


if __name__ == "__main__":
    forearm_roll_drive()   # build: writes the sibling forearm_roll_drive.step
