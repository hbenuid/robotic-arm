"""gripper - the rack & pinion gripper (PapaBravo's "Rack & Pinion Robotic Gripper Jaw", Printables, CC BY 4.0), a
code-driven module like the wrist roll drive; the arm places it through lib/mounts.py MODULE_MOUNTS ("gripper#1" on
wrist_link#1, on the wrist roll's axis R' at gripper_j3_connector's plate).

gripper_j3_connector bolts to the wrist roll pulley's face; the cage (gripper_cage) bolts onto its plate and holds the
MG996R upside down by its tabs on the base's posts (gripper_base), the pinion (gripper_pinion) pressed on its spline
over the base's plate; the two racks (gripper_rack, a and b - b half a turn round the axis) run on the plate between
its ribs and walls, their posts down through its slots into the jaws (gripper_jaw, a and b) under it: the pinion drives
them apart and together. MODULE FRAME (lib/gripper/layout.py): origin on R' on the connector's plate's gripper face, +Z
along R' toward the fingertips, +Y along the jaws' travel; at zero the racks are centred (the jaws 23 apart). Every
placement comes from lib/gripper/layout.py rows() - never type one here. The tab screws, the racks' screws, their nuts
and the cage's screws onto the connector are tools/bom.py EXTRAS.
"""
from cad import step

from assemblies._occurrences import located_children
from lib import reference as R
from lib.assembly import assembly
from lib.gripper.layout import rows
from lib.models import raw

# (part, role | None, placement in the module frame): lib/gripper/layout.py rows()
OCCURRENCES = rows()

# The module's rigid bodies (robot/frames.py LINKS: "gripper#1:base" in wrist_roll_link - the pinion turns, but with
# the jaws it drives, not on a joint of its own -, ":jaw_a" / ":jaw_b" in the jaw links); a member is a part, or a
# part:role when the module has the part twice.
BODIES = {
    "base": frozenset({"gripper_j3_connector", "gripper_cage", "mg996r_servo", "gripper_base", "gripper_pinion"}),
    "jaw_a": frozenset({"gripper_rack:a", "gripper_jaw:a"}),
    "jaw_b": frozenset({"gripper_rack:b", "gripper_jaw:b"}),
}

# Totals lock (tests/test_assembly.py, test_robot.py; robot/ inertials sum the same rows): whole module + per body.
# Re-derive with totals() / totals("base") after any geometry change.
EXPECTED = {
    "leaves": 9, "solids": 12, "solid_volume": 87501.433,
    "bodies": {"base": {"leaves": 5, "solids": 8, "solid_volume": 70269.29},
               "jaw_a": {"leaves": 2, "solids": 2, "solid_volume": 8616.072},
               "jaw_b": {"leaves": 2, "solids": 2, "solid_volume": 8616.072}},
}


def in_body(label: str, body: str) -> bool:
    """Whether a child labelled `label` (part or part:role) is in BODIES[body]."""
    return label in BODIES[body] or label.split(":")[0] in BODIES[body]


@step(out="gripper.step")
def gripper():
    """The gripper in its module frame, every part in its colour as in the arm (_occurrences.colour())."""
    return assembly("gripper", located_children(OCCURRENCES, tint=True))


def totals(body: str | None = None, *, shape=None) -> dict:
    """(leaves, solids, solid_volume) of a fresh in-process build - or of `shape`, a build of it already made - the
    whole module or one of BODIES."""
    module = raw(gripper) if shape is None else shape
    leaves = [c for c in module.children if body is None or in_body(c.label, body)]
    return {"leaves": len(leaves), "solids": sum(len(c.solids()) for c in leaves),
            "solid_volume": round(sum(R.solid_volume(c) for c in leaves), 3)}


if __name__ == "__main__":
    gripper()   # build: writes the sibling gripper.step
