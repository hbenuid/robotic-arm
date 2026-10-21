"""wrist_roll_drive - the belt-driven wrist roll, a code-driven module like the forearm roll drive, one joint further out.

Stator: the frame is wrist_link itself (its DEFAULT build: the plate with j3_coupler#2's seat and, on the plate's
underside, one body round the pancake and the tower round the roll axis - lib/wrist/roll.py build_frame), so it is the
module's host, not a row; the rows: the NEMA 17 pancake ON the wrist-pitch axis in the frame's pocket (open on the far
face, the pocket's front wall the plate it bolts to, slotted along Y for the belt's tension) with its MKS SERVO42D on
its rear face, in a window through the frame's rear, and the 20T on its shaft, its hub down in the plate's pilot slot;
the 6806 pair back to back on the tower's lip; the wrist roll belt (no idler yet:
docs/open_issues.md). Rotor: the pulley (wrist_roll_pulley, the output: its 40T band beside the pancake's 20T, its hub
through bearing 2, the face gripper_j3_connector bolts to - 4x M3 into nuts in its rim) and the shaft (wrist_roll_shaft,
a collar: its hub through bearing 1 to the pulley's, the rotor clamp's nuts and the stop lug in its flange), clamped by
4x M3 from the pulley's face; the gripper servo's cable runs through both. MODULE FRAME (lib/wrist/params.py
WristRollParams): origin on the roll axis R' at the pitch axis' station, +Z along R' toward the gripper, +X away from
the plate (wrist_link's -Z), +Y from the pancake toward R', the pitch axis along X at y_motor (the arm's wrist offset
across R'); the arm places it through lib/mounts.py MODULE_MOUNTS ("wrist_roll_drive#1" on wrist_link#1). Every
station comes from lib/wrist stack_positions() - never type one here. Gear ratio 40 / 20 = 2 (WRIST_ROLL_RATIO) on the
belt (wrist_roll_belt, WRIST_ROLL_BELT_LENGTH); the rotor clamp's and the connector's M3 screws + nuts and the
pancake's M3 are tools/bom.py EXTRAS.
"""
from cadgen import step

from assemblies._occurrences import located_children
from lib import reference as R
from lib.assembly import assembly
from lib.models import raw
from lib.wrist import DEFAULT
from lib.wrist.layout import pancake_location, stack_positions

S = stack_positions(DEFAULT)

# The module's printed parts' colour (assemblies/arm.py MODULE_TINTS reuses it); purchased parts are BOUGHT_TINT.
TINT = "#B07AA1"


def _at(x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (x, y, z)


# (part, role | None, placement in the module frame) - a position, or ((x, y, z), (rx, ry, rz)) when the part's own
# frame is not the module's. The shaft and the pulley are built at their stations (their rows are at 0); the bearings
# stand on their seats; the pancake's mounting face is the plate's rear face, the pancake on the pitch axis, spun so its
# plug points +X (the open face).
OCCURRENCES = [
    # every purchased row carries the joint in its role: the labels stay unique in the arm (the forearm roll drive's
    # pair is bearing_6806:1 / :2)
    ("bearing_6806",          "wrist_roll_1", _at(z=S["z_bearing_1"])),
    ("wrist_roll_shaft",      None, _at()),
    ("bearing_6806",          "wrist_roll_2", _at(z=S["z_bearing_2"])),
    ("wrist_roll_pulley",     None, _at()),
    # the pancake, its board, its 20T and the belt - on the pitch axis (-Y); the board on the pancake's rear face, turned
    # with it (its motor plug toward the pancake's, at the open face), in the frame's rear window
    ("nema17_pancake",        "wrist_roll", pancake_location(DEFAULT)),
    ("mks_servo42d",          "wrist_roll", ((S["x_motor"], S["y_motor"], S["z_motor_rear"]), (0.0, 0.0, DEFAULT.roll.motor_spin_deg))),
    ("gt2_pulley_20t",        "wrist_roll", ((S["x_motor"], S["y_motor"], S["z_20t"]), (0.0, -90.0, 0.0))),   # hub face pulley_seat over the pilot, bore axis along +Z
    ("wrist_roll_belt",       "wrist_roll", ((S["x_motor"], S["y_motor"], S["z_ring_mid"]), (0.0, 0.0, 90.0))),   # the 20T to the 40T: its frame's +X along module +Y
]

# The module's rigid bodies (robot/frames.py LINKS: "wrist_roll_drive#1:stator" in wrist_pitch_link, ":rotor" in
# wrist_roll_link - with the gripper).
ROTOR = frozenset({"wrist_roll_shaft", "wrist_roll_pulley"})
BODIES = {"rotor": ROTOR, "stator": frozenset(part for part, _, _ in OCCURRENCES) - ROTOR}

# Totals lock (tests/test_assembly.py, test_robot.py; robot/ inertials sum the same rows): whole module + per body.
# Re-derive with totals() / totals("stator") after any geometry change.
EXPECTED = {
    "leaves": 8, "solids": 32, "solid_volume": 84570.171,
    "bodies": {"stator": {"leaves": 6, "solids": 30, "solid_volume": 58770.592},
               "rotor": {"leaves": 2, "solids": 2, "solid_volume": 25799.58}},
}


@step
def wrist_roll_drive():
    """The drive in its module frame (printed parts TINT, purchased parts BOUGHT_TINT)."""
    return assembly("wrist_roll_drive", located_children(OCCURRENCES, tint=TINT))


def totals(body: str | None = None, *, shape=None) -> dict:
    """(leaves, solids, solid_volume) of a fresh in-process build - or of `shape`, a build of it already made - the
    whole module or one of BODIES."""
    module = raw(wrist_roll_drive) if shape is None else shape
    leaves = [c for c in module.children if body is None or c.label.split(":")[0] in BODIES[body]]
    return {"leaves": len(leaves), "solids": sum(len(c.solids()) for c in leaves),
            "solid_volume": round(sum(R.solid_volume(c) for c in leaves), 3)}


if __name__ == "__main__":
    wrist_roll_drive()   # build: writes the sibling wrist_roll_drive.step
