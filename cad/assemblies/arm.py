"""The whole arm: every part of the SolidWorks 'final Arm Assembly Fully Movable' placed from
reference/placements.json - the SolidWorks part occurrences + the mounted ones (lib/mounts.py: the belt joints'
NEMA 17 motors - 48 mm at the base, 40 mm at the elbow and wrist - and their MKS SERVO42D boards, on the pads the links carry;
the 6806-2RS bearing pair in each belt joint's bore; the base_yaw thrust bearing in the base's groove; the elbow's and the wrist's 90T re-seated on them
and the base_yaw 120T under j1_coupler, each clamped by 4x M4 screws + nuts) + the gripper module
(SolidWorks-driven) + the cycloidal_drive module (code-driven, placed at the SolidWorks node's pose) + the
forearm_roll_drive module (code-driven, placed by lib/mounts.py MODULE_MOUNTS) - the totals are locked in
tests/test_assembly.py - bucketed into the GROUPS component tree (arm -> base_link/shoulder_link/upper_arm_link/
elbow_link/forearm_link/wrist_pitch_link/wrist) so each rigid link toggles as one node in the viewers. Colors: a PRINTED part carries its link's
(or its module's) tint, every PURCHASED part (COTS = True - motors, boards, servo, bearings, pins, bolts, nuts,
rails, the 20T pulleys) is _occurrences.BOUGHT_TINT grey, inside the modules too:

    arm
    |- base_link         base, bearing_6806:base_yaw_1 / _2, washer_as6590:base_yaw_1, bearing_axk6590:base_yaw,
    |                    base_motor_mount, base_motor_mount_screws / _nuts:base_yaw, nema17_48mm:base_yaw,
    |                    mks_servo42d:base_yaw
    |- shoulder_link     j1_coupler, washer_as6590:base_yaw_2, gt2_pulley_120t:base_yaw, yaw_pulley_screws / _nuts:base_yaw,
    |                    cycloidal_drive (kept whole - see below)
    |- upper_arm_link    j1_link, bearing_6806:elbow_pitch_1 / _2, nema17_40mm:elbow_pitch, mks_servo42d:elbow_pitch
    |- elbow_link        gt2_pulley_90t:j2, elbow_pulley_screws / _nuts:elbow_pitch, forearm_roll_drive (kept whole - see
    |                    below; its block IS the elbow coupler)
    |- forearm_link      j2_link, bearing_6806:wrist_pitch_1 / _2, nema17_40mm:wrist_pitch, mks_servo42d:wrist_pitch
    |- wrist_pitch_link  gt2_pulley_90t:j3, wrist_pulley_screws / _nuts:wrist_pitch, j3_coupler:j3, wrist_link,
    |                    gripper_clamp_bracket, nema17_pancake
    |- wrist             gt2_pulley_20t, gripper

j3_coupler#1 (the elbow's SolidWorks coupler) is RETIRED (lib/placements.py): the roll drive's block carries its
lip / boss / journal / stub and the elbow 90T bolts straight into it - the record stays in placements.json, no
table claims it; likewise gt2_pulley_90t#1 / #2, whose re-seated poses are the mounts gt2_pulley_90t#3 / #4. The SolidWorks link caps (j1_cap, j2_cap_1, j2_cap_2) were removed 2026-09-25: their products are
lib/reference.py SKIPPED_PRODUCTS, their poses sit in placements.json `skipped`. A drive module is one linked child, so this tree keeps it whole - the cycloidal drive under shoulder_link
although its rotor body (output hub + pins) belongs to upper_arm_link, the forearm roll drive under elbow_link
although its rotor (the roll shaft) belongs to forearm_link - in robot/frames.py LINKS, the kinematic truth,
which the per-link meshes follow (each in its own MODULE_TINTS colour).

Frame: the placements are in the SolidWorks capture frame W (+Y up), but the arm is EMITTED in
the base_link frame B (lib/datum.py base_frame(), the frame robot/frames.py gives base_link - REP-103:
Z up, X forward, the base's mounting face on z = 0) - arm_from_w(), composed into every occurrence's placement. cadgen's viewer and snapshots
are Z-up with no up-axis option, so a W-frame STEP renders lying on its side; in B arm.step and
robot/arm.urdf open in the same pose. placements.json, the parts and the modules stay in W.

Run:  ./cadtool gen assemblies/arm.py            -> assemblies/arm.step (git-ignored); every stale
                                                  child part is rebuilt and its (git-ignored) STEP rewritten
      ./cadtool inspect assemblies/arm.step      -> leaf refs, solids, faces, volume, bbox
      ./cadtool viewer                           -> http://127.0.0.1:3245/?file=assemblies/arm.step
"""

from cadgen import step

from assemblies import cycloidal_drive, forearm_roll_drive, gripper
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
ROLL_KEY = "forearm_roll_drive#1"
MODULE_KEYS = (DRIVE_KEY, ROLL_KEY, GRIPPER_KEY)

# (part or module, role, placements.json key) in SolidWorks document order, each mounted bearing pair, motor + board
# (lib/mounts.py, role = the joint it serves; a bearing's _1 / _2 = the coupler / pulley side) right after its host,
# a re-seated pulley where its retired SolidWorks record stood. The j2/j3 roles of the duplicated pulley (+ the
# wrist's coupler) are positional (elbow_pitch vs wrist_pitch, in that order).
OCCURRENCES = [
    ("base",                  None, "base#1"),
    ("bearing_6806",          "base_yaw_1", "bearing_6806#1"),     # mounted: the base's bearing bore
    ("bearing_6806",          "base_yaw_2", "bearing_6806#2"),
    ("washer_as6590",         "base_yaw_1", "washer_as6590#1"),    # mounted: the thrust bearing in the base's groove ...
    ("bearing_axk6590",       "base_yaw", "bearing_axk6590#1"),
    ("base_motor_mount",      None, "base_motor_mount#1"),         # mounted: the base's bolt-on motor lobe ...
    ("base_motor_mount_screws", "base_yaw", "base_motor_mount_screws#1"),   # ... its 4x M4 ...
    ("base_motor_mount_nuts", "base_yaw", "base_motor_mount_nuts#1"),       # ... into the nuts in the base's ribs
    ("nema17_48mm",           "base_yaw", "nema17_48mm#1"),        # mounted: the 48 mm motor under the motor mount's plate
    ("mks_servo42d",          "base_yaw", "mks_servo42d#1"),
    ("j1_coupler",            None, "j1_coupler#1"),
    ("washer_as6590",         "base_yaw_2", "washer_as6590#2"),    # ... its upper washer under j1_coupler's seat
    ("gt2_pulley_120t",       "base_yaw", "gt2_pulley_120t#1"),    # mounted: the base_yaw 120T on j1_coupler's stub end, in the lower base bearing
    ("yaw_pulley_screws",     "base_yaw", "yaw_pulley_screws#1"),  # mounted: the base_yaw 120T's 4x M4 ...
    ("yaw_pulley_nuts",       "base_yaw", "yaw_pulley_nuts#1"),    # ... into the nuts in j1_coupler's hub
    ("cycloidal_drive",       None, DRIVE_KEY),            # module: assemblies/cycloidal_drive.py (the shoulder_pitch joint)
    ("j1_link",               None, "j1_link#1"),
    ("bearing_6806",          "elbow_pitch_1", "bearing_6806#3"),  # mounted: j1_link's elbow bore
    ("bearing_6806",          "elbow_pitch_2", "bearing_6806#4"),
    ("nema17_40mm",           "elbow_pitch", "nema17_40mm#2"),     # mounted: j1_link's pad
    ("mks_servo42d",          "elbow_pitch", "mks_servo42d#2"),
    ("gt2_pulley_90t",        "j2", "gt2_pulley_90t#3"),           # mounted: re-seated on the lower elbow bearing (#1 retired)
    ("elbow_pulley_screws",   "elbow_pitch", "elbow_pulley_screws#1"),   # mounted: the elbow 90T's 4x M4 ...
    ("elbow_pulley_nuts",     "elbow_pitch", "elbow_pulley_nuts#1"),     # ... into the block's captive nuts
    # j3_coupler#1 (the elbow coupler) is retired: the roll drive's block bolts to the elbow pulley in its place
    ("forearm_roll_drive",    None, ROLL_KEY),             # module: assemblies/forearm_roll_drive.py (the forearm_roll joint; lib/mounts.py MODULE_MOUNTS)
    ("j2_link",               None, "j2_link#1"),
    ("bearing_6806",          "wrist_pitch_1", "bearing_6806#5"),  # mounted: j2_link's wrist boss
    ("bearing_6806",          "wrist_pitch_2", "bearing_6806#6"),
    ("nema17_40mm",           "wrist_pitch", "nema17_40mm#3"),     # mounted: j2_link's web slots
    ("mks_servo42d",          "wrist_pitch", "mks_servo42d#3"),
    ("gt2_pulley_90t",        "j3", "gt2_pulley_90t#4"),           # mounted: re-seated on the lower wrist bearing (#2 retired)
    ("wrist_pulley_screws",   "wrist_pitch", "wrist_pulley_screws#1"),   # mounted: the wrist 90T's 4x M4 ...
    ("wrist_pulley_nuts",     "wrist_pitch", "wrist_pulley_nuts#1"),     # ... into the nuts in j3_coupler's flange
    ("j3_coupler",            "j3", "j3_coupler#2"),
    ("wrist_link",            None, "wrist_link#1"),
    ("gripper_clamp_bracket", None, "gripper_clamp_bracket#1"),
    ("nema17_pancake",        None, "nema17_pancake#1"),
    ("gt2_pulley_20t",        None, "gt2_pulley_20t#1"),
    ("gripper",               None, GRIPPER_KEY),          # module: assemblies/gripper.py
]

MODULES = {"gripper": gripper.gripper, "cycloidal_drive": cycloidal_drive.cycloidal_drive,
           "forearm_roll_drive": forearm_roll_drive.forearm_roll_drive}   # the child MODELS

# The component tree: the rigid-link partition of robot/frames.py LINKS with the three modules
# kept whole (wrist = wrist_roll_link + jaw_a_link + jaw_b_link; the cycloidal drive under shoulder_link;
# the forearm roll drive under elbow_link although LINKS puts its rotor in forearm_link).
# Rows are (group label, tint, occurrence keys in document order); tests lock the LINKS mirror.
# The tints color the PRINTED parts; grey is reserved for the purchased ones (BOUGHT_TINT).
GROUPS = [
    ("base_link",        "#937860", ("base#1", "bearing_6806#1", "bearing_6806#2", "washer_as6590#1", "bearing_axk6590#1",
                                     "base_motor_mount#1", "base_motor_mount_screws#1", "base_motor_mount_nuts#1",
                                     "nema17_48mm#1", "mks_servo42d#1")),
    ("shoulder_link",    "#4C72B0", ("j1_coupler#1", "washer_as6590#2", "gt2_pulley_120t#1", "yaw_pulley_screws#1", "yaw_pulley_nuts#1",
                                     DRIVE_KEY)),
    ("upper_arm_link",   "#CCB974", ("j1_link#1", "bearing_6806#3", "bearing_6806#4", "nema17_40mm#2", "mks_servo42d#2")),
    ("elbow_link",       "#DA8BC3", ("gt2_pulley_90t#3", "elbow_pulley_screws#1", "elbow_pulley_nuts#1", ROLL_KEY)),
    ("forearm_link",     "#DD8452", ("j2_link#1", "bearing_6806#5", "bearing_6806#6", "nema17_40mm#3", "mks_servo42d#3")),
    ("wrist_pitch_link", "#55A868", ("gt2_pulley_90t#4", "wrist_pulley_screws#1", "wrist_pulley_nuts#1", "j3_coupler#2", "wrist_link#1",
                                     "gripper_clamp_bracket#1", "nema17_pancake#1")),
    ("wrist",            "#8172B3", ("gt2_pulley_20t#1", GRIPPER_KEY)),
]
MODULE_TINTS = {DRIVE_KEY: cycloidal_drive.TINT, ROLL_KEY: forearm_roll_drive.TINT, GRIPPER_KEY: gripper.TINT}  # the modules keep their own color in their group


@step
def arm():
    """The arm as a labelled Compound: 'arm' -> the GROUPS component nodes -> parts + the
    'gripper' and 'cycloidal_drive' modules, printed parts tinted with their group's color and
    purchased parts BOUGHT_TINT grey, in the base_link frame (Z up - arm_from_w())."""
    return assembly("arm", grouped_children(OCCURRENCES, GROUPS, MODULES, MODULE_TINTS, root=arm_from_w()))


if __name__ == "__main__":
    arm()   # build: writes the sibling arm.step
