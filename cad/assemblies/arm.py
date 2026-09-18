"""The whole arm: every part of the SolidWorks 'final Arm Assembly Fully Movable' placed from
reference/placements.json - 15 part occurrences + the gripper module (SolidWorks-driven)
+ the cycloidal_drive module (code-driven, placed at the SolidWorks node's pose) = 52 leaves,
bucketed into the GROUPS component tree (arm -> base_link/shoulder_link/upper_arm_link/
forearm_link/wrist_pitch_link/wrist) so each rigid link toggles as one node in the viewers:

    arm
    |- base_link         base
    |- shoulder_link     j1_coupler, cycloidal_drive (18, kept whole - see below)
    |- upper_arm_link    j1_link, j1_cap
    |- forearm_link      gt2_pulley_90t:j2, j3_coupler:j2, j2_link, j2_cap_1, j2_cap_2
    |- wrist_pitch_link  gt2_pulley_90t:j3, j3_coupler:j3, wrist_link, gripper_clamp_bracket, nema17_pancake
    |- wrist             gt2_pulley_20t, gripper (19)

The drive module is one linked child, so this tree keeps it whole under shoulder_link (in its
own MODULE_TINTS colour) although its rotor body (output hub + pins) belongs to upper_arm_link
in robot/frames.py LINKS - the kinematic truth, which the per-link meshes follow.

Frame: the placements are in the SolidWorks capture frame W (+Y up), but the arm is EMITTED in
the base_link frame B (lib/datum.py base_frame(), the frame robot/frames.py gives base_link - REP-103:
Z up, X forward, the base's mounting face on z = 0) - arm_from_w(), composed into every occurrence's placement. cadgen's viewer and snapshots
are Z-up with no up-axis option, so a W-frame STEP renders lying on its side; in B arm.step and
robot/arm.urdf open in the same pose. placements.json, the parts and the modules stay in W.

Run:  ./cadtool gen assemblies/arm.py            -> assemblies/arm.step (git-ignored); every stale
                                                  child part is rebuilt and its committed STEP rewritten
      ./cadtool inspect assemblies/arm.step      -> leaf refs, solids, faces, volume, bbox
      ./cadtool show assemblies/arm.py           -> preview in the OCP CAD Viewer (no build)
"""

from cadgen import step

from assemblies import cycloidal_drive, gripper
from assemblies._occurrences import grouped_children
from lib.assembly import assembly
from lib.datum import base_frame

def arm_from_w():
    """W (SolidWorks capture frame, +Y up, arm toward -X) -> base_link frame B (REP-103: Z up, X
    forward, origin on the base's mounting face at the base_yaw axis): the frame the arm is emitted in.
    A function: a Location at module level would import the kernel with this model file."""
    return base_frame().inverse()


GRIPPER_KEY = "gripper#1"
DRIVE_KEY = "cycloidal_drive#1"
MODULE_KEYS = (DRIVE_KEY, GRIPPER_KEY)

# (part or module, role, placements.json key) in SolidWorks document order. The j2/j3 roles
# of the duplicated pulley + coupler are positional (elbow_pitch vs wrist_pitch, in that order).
OCCURRENCES = [
    ("base",                  None, "base#1"),
    ("j1_coupler",            None, "j1_coupler#1"),
    ("cycloidal_drive",       None, DRIVE_KEY),            # module: assemblies/cycloidal_drive.py (the shoulder_pitch joint)
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

MODULES = {"gripper": gripper.gripper, "cycloidal_drive": cycloidal_drive.cycloidal_drive}   # the child MODELS

# The component tree: the rigid-link partition of robot/frames.py LINKS with the two modules
# kept whole (wrist = wrist_roll_link + jaw_a_link + jaw_b_link; the drive under shoulder_link).
# Rows are (group label, tint, occurrence keys in document order); tests lock the LINKS mirror.
GROUPS = [
    ("base_link",        "#8C8C8C", ("base#1",)),
    ("shoulder_link",    "#4C72B0", ("j1_coupler#1", DRIVE_KEY)),
    ("upper_arm_link",   "#CCB974", ("j1_link#1", "j1_cap#1")),
    ("forearm_link",     "#DD8452", ("gt2_pulley_90t#1", "j3_coupler#1", "j2_link#1", "j2_cap_1#1", "j2_cap_2#1")),
    ("wrist_pitch_link", "#55A868", ("gt2_pulley_90t#2", "j3_coupler#2", "wrist_link#1", "gripper_clamp_bracket#1", "nema17_pancake#1")),
    ("wrist",            "#8172B3", ("gt2_pulley_20t#1", GRIPPER_KEY)),
]
MODULE_TINTS = {DRIVE_KEY: "#C44E52", GRIPPER_KEY: "#64B5CD"}  # the named modules stay distinct in their group


@step
def arm():
    """The arm as a labelled Compound: 'arm' -> the GROUPS component nodes -> parts + the
    'gripper' and 'cycloidal_drive' modules, each subtree tinted with its group's color, in the
    base_link frame (Z up - arm_from_w())."""
    return assembly("arm", grouped_children(OCCURRENCES, GROUPS, MODULES, MODULE_TINTS, root=arm_from_w()))


if __name__ == "__main__":
    arm()   # build: writes the sibling arm.step (preview: ./cadtool show assemblies/arm.py)
