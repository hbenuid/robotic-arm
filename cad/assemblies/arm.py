"""The whole arm: every part of the SolidWorks 'final Arm Assembly Fully Movable' placed from
reference/placements.json - 16 top-level occurrences + the gripper module (SolidWorks-driven)
+ the cycloidal_drive module (code-driven, placed at the SolidWorks node's pose) = 52 leaves.

Run:  ./cadtool step assemblies/arm.py           -> assemblies/arm.step (git-ignored)
      ./cadtool inspect refs assemblies/arm.step --facts --planes --positioning
      ./cadtool python -m assemblies.arm         -> preview in the OCP CAD Viewer
"""
# --- path shim: files inside assemblies/ -> parent.parent (= cad/) --------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from assemblies import cycloidal_drive, gripper  # noqa: E402
from assemblies._occurrences import add_occurrences  # noqa: E402
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


def gen_step():
    """The arm as a labelled Compound: 'arm' -> parts + the 'gripper' and 'cycloidal_drive' modules."""
    asm = AssemblyHelper("arm")
    add_occurrences(asm, OCCURRENCES, MODULES)
    return asm.build()


if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
