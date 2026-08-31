"""The whole arm: every part of the SolidWorks 'final Arm Assembly Fully Movable' placed from
reference/placements.json - 15 part occurrences + the gripper module (SolidWorks-driven)
+ the cycloidal_drive module (code-driven, placed at the SolidWorks node's pose) = 52 leaves,
bucketed into the GROUPS component tree (arm -> base_link/link1/link2/link3/wrist) so each
rigid link toggles as one node in the viewers:

    arm
    |- base_link   base
    |- link1       j1_coupler, cycloidal_drive (18), j1_link, j1_cap
    |- link2       gt2_pulley_90t:j2, j3_coupler:j2, j2_link, j2_cap_1, j2_cap_2
    |- link3       gt2_pulley_90t:j3, j3_coupler:j3, wrist_link, gripper_clamp_bracket, nema17_pancake
    |- wrist       gt2_pulley_20t, gripper (19)

Run:  ./cadtool step assemblies/arm.py           -> assemblies/arm.step (git-ignored)
      ./cadtool inspect refs assemblies/arm.step --facts --planes --positioning
      ./cadtool python -m assemblies.arm         -> preview in the OCP CAD Viewer
"""
# --- path shim: files inside assemblies/ -> parent.parent (= cad/) --------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from assemblies import cycloidal_drive, gripper  # noqa: E402
from assemblies._occurrences import add_grouped_occurrences  # noqa: E402
from lib.assembly import AssemblyHelper  # noqa: E402

GRIPPER_KEY = "gripper#1"
DRIVE_KEY = "cycloidal_drive#1"
MODULE_KEYS = (DRIVE_KEY, GRIPPER_KEY)

# (part or module, role, placements.json key) in SolidWorks document order. The J2/J3 roles
# of the duplicated pulley + coupler come from their world positions (shoulder vs elbow).
OCCURRENCES = [
    ("base",                  None, "base#1"),
    ("j1_coupler",            None, "j1_coupler#1"),
    ("cycloidal_drive",       None, DRIVE_KEY),            # module: assemblies/cycloidal_drive.py (shoulder pitch actuator)
    ("j1_link",               None, "j1_link#1"),
    ("gt2_pulley_90t",        "j2", "gt2_pulley_90t#1"),
    ("j3_coupler",            "j2", "j3_coupler#1"),
    ("j2_link",               None, "j2_link#1"),
    ("gt2_pulley_90t",        "j3", "gt2_pulley_90t#2"),
    ("j3_coupler",            "j3", "j3_coupler#2"),
    ("wrist_link",            None, "wrist_link#1"),
    ("gripper_clamp_bracket", None, "gripper_clamp_bracket#1"),
    ("nema17_pancake",        None, "nema17_pancake#1"),
    ("gt2_pulley_20t",        None, "gt2_pulley_20t#1"),
    ("gripper",               None, GRIPPER_KEY),          # module: assemblies/gripper.py
    ("j2_cap_1",              None, "j2_cap_1#1"),
    ("j2_cap_2",              None, "j2_cap_2#1"),
    ("j1_cap",                None, "j1_cap#1"),
]

MODULES = {"gripper": gripper.gen_step, "cycloidal_drive": cycloidal_drive.gen_step}

# The component tree: the rigid-link partition of robot/frames.py LINKS with the gripper module
# kept whole (wrist = wrist_roll_link + jaw_a_link + jaw_b_link). Rows are
# (group label, tint, occurrence keys in document order); tests lock the LINKS mirror.
GROUPS = [
    ("base_link", "#8C8C8C", ("base#1",)),
    ("link1",     "#4C72B0", ("j1_coupler#1", DRIVE_KEY, "j1_link#1", "j1_cap#1")),
    ("link2",     "#DD8452", ("gt2_pulley_90t#1", "j3_coupler#1", "j2_link#1", "j2_cap_1#1", "j2_cap_2#1")),
    ("link3",     "#55A868", ("gt2_pulley_90t#2", "j3_coupler#2", "wrist_link#1", "gripper_clamp_bracket#1", "nema17_pancake#1")),
    ("wrist",     "#8172B3", ("gt2_pulley_20t#1", GRIPPER_KEY)),
]
MODULE_TINTS = {DRIVE_KEY: "#C44E52", GRIPPER_KEY: "#64B5CD"}  # the named modules stay distinct in their group


def gen_step():
    """The arm as a labelled Compound: 'arm' -> the GROUPS component nodes -> parts + the
    'gripper' and 'cycloidal_drive' modules, each subtree tinted with its group's color."""
    asm = AssemblyHelper("arm")
    add_grouped_occurrences(asm, OCCURRENCES, GROUPS, MODULES, MODULE_TINTS)
    return asm.build()


if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
