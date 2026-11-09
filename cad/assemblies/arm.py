"""The whole arm: every part of the SolidWorks 'final Arm Assembly Fully Movable' placed from
reference/placements.json - the SolidWorks part occurrences + the mounted ones (lib/mounts.py: the belt joints'
NEMA 17 motors - 48 mm at the base, 40 mm at the elbow and wrist - and their MKS SERVO42D boards, on the pads the links carry,
the elbow's on its 5:1 planetary gearbox;
the 6806-2RS bearing pair in each belt joint's bore; the base_yaw thrust bearing in the base's groove; the elbow's and the wrist's 90T re-seated on them
and the base_yaw 120T under j1_coupler, each clamped by 4x M4 screws + nuts; j1_coupler's motor leg) + the cycloidal_drive
module (code-driven, placed at the SolidWorks node's pose) + the forearm_roll_drive, wrist_roll_drive and gripper
modules (code-driven, placed by lib/mounts.py MODULE_MOUNTS) - the totals are locked in
tests/test_assembly.py - bucketed into the GROUPS component tree (arm -> base_link/shoulder_link/upper_arm_link/
elbow_link/forearm_link/wrist_pitch_link/wrist) so each rigid link toggles as one node in the viewers. Colors
(_occurrences.colour(), lib/style.py): a PRINTED part is ivory, or orange where it drives, turns or covers
(ACCENT = True), every PURCHASED part (COTS = True - motors, the elbow's gearbox, boards, servo, bearings, pins, bolts,
nuts, the 20T pulleys) is grey and the belts a darker grey, inside the modules too:

    arm
    |- base_link         base, bearing_6806:base_yaw_1 / _2, washer_as6590:base_yaw_1, bearing_axk6590:base_yaw,
    |                    base_motor_mount, base_motor_mount_screws / _nuts:base_yaw, nema17_48mm:base_yaw,
    |                    mks_servo42d:base_yaw, yaw_motor_screws:base_yaw, gt2_pulley_20t:base_yaw, yaw_belt:base_yaw,
    |                    gt2_idler_20t / yaw_idler_spacer / _screw / _nut:base_yaw,
    |                    ky003_hall_sensor / yaw_home_bracket / _screws / _nuts / _board_screws / _board_nuts:base_yaw
    |- shoulder_link     j1_coupler, j1_motor_leg, washer_as6590:base_yaw_2, gt2_pulley_120t:base_yaw,
    |                    yaw_pulley_screws / _nuts:base_yaw, yaw_home_magnet:base_yaw, cycloidal_drive (kept whole - see below)
    |- upper_arm_link    j1_link, bearing_6806:elbow_pitch_1 / _2, eg17_g5_gearbox:elbow_pitch,
    |                    elbow_gearbox_screws:elbow_pitch, nema17_40mm:elbow_pitch, mks_servo42d:elbow_pitch,
    |                    elbow_motor_screws:elbow_pitch, gt2_pulley_20t_8mm:elbow_pitch, elbow_belt:elbow_pitch,
    |                    gt2_idler_20t / elbow_idler_spacer / _screw / _nut:elbow_pitch
    |- elbow_link        gt2_pulley_90t:j2, elbow_pulley_screws / _nuts:elbow_pitch, forearm_roll_drive (kept whole - see
    |                    below; its block IS the elbow coupler)
    |- forearm_link      j2_link, bearing_6806:wrist_pitch_1 / _2, nema17_40mm:wrist_pitch, mks_servo42d:wrist_pitch,
    |                    gt2_pulley_20t:wrist_pitch, wrist_motor_screws:wrist_pitch, wrist_belt:wrist_pitch,
    |                    gt2_idler_20t / wrist_idler_screw / _nut:wrist_pitch
    |- wrist_pitch_link  gt2_pulley_90t:j3, wrist_pulley_screws / _nuts:wrist_pitch, wrist_link (the wrist roll drive's
    |                    frame, the wrist_pitch output flange on its plate), wrist_roll_drive (kept whole - see below)
    |- wrist             gripper

j3_coupler#1 / #2 (the elbow's and the wrist's SolidWorks coupler) are RETIRED (lib/placements.py): the roll drive's
block and wrist_link carry its journal (a post) and stub and each 90T bolts straight into them - the records stay in
placements.json, no table claims them; likewise gt2_pulley_90t#1 / #2, whose re-seated poses are the mounts gt2_pulley_90t#3 / #4, and
nema17_pancake#1 / gt2_pulley_20t#1, which the wrist roll drive carries (the pancake on the wrist-pitch axis). The
SolidWorks link caps (j1_cap, j2_cap_1, j2_cap_2) were removed 2026-09-25, gripper_clamp_bracket 2026-10-14 and the
SolidWorks gripper 2026-11-07 (the rack & pinion gripper in its place): their products are lib/reference.py SKIPPED_PRODUCTS, their poses sit in placements.json `skipped`. A drive module is one
inlined, tinted copy, so this tree keeps it whole - the cycloidal drive under shoulder_link although its rotor body
(the turning shell: shell ring, ring pins, housing bolts + nuts, both 6814s) belongs to upper_arm_link, the forearm roll drive under elbow_link although its rotor (the roll pulley +
shaft) belongs to forearm_link, the wrist roll drive under wrist_pitch_link although its rotor (its pulley + shaft)
belongs to wrist_roll_link - in robot/frames.py LINKS, the kinematic truth, which the per-link meshes follow.

Frame: the placements are in the SolidWorks capture frame W (+Y up), but the arm is EMITTED in
the base_link frame B (lib/datum.py base_frame(), the frame robot/frames.py gives base_link - REP-103:
Z up, X forward, the base's mounting face on z = 0) - arm_from_w(), composed into every occurrence's placement. cadgen's viewer and snapshots
are Z-up with no up-axis option, so a W-frame STEP renders lying on its side; in B arm.step and
robot/arm.urdf open in the same pose. placements.json, the parts and the modules stay in W.

Run:  ./cadtool gen assemblies/arm.py            -> assemblies/arm.step (git-ignored); every child part runs
                                                  with it (replayed when unchanged) and writes its (git-ignored) STEP
      ./cadtool inspect assemblies/arm.step      -> leaf refs, solids, faces, volume, bbox
      ./cadtool viewer                           -> the arm's page on the machine's cad server
"""

from cadgen import step

from assemblies import cycloidal_drive, forearm_roll_drive, gripper, wrist_roll_drive
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
WRIST_ROLL_KEY = "wrist_roll_drive#1"
MODULE_KEYS = (DRIVE_KEY, ROLL_KEY, WRIST_ROLL_KEY, GRIPPER_KEY)

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
    ("yaw_motor_screws",      "base_yaw", "yaw_motor_screws#1"),   # mounted: the base_yaw motor's 4x M3, down through the plate
    ("gt2_pulley_20t",        "base_yaw", "gt2_pulley_20t#3"),     # mounted: on the base_yaw motor's shaft, level with the 120T ...
    ("yaw_belt",              "base_yaw", "yaw_belt#1"),           # ... and its belt to it, ...
    ("gt2_idler_20t",         "base_yaw", "gt2_idler_20t#2"),      # mounted: ... its tensioner on the motor mount's plate, ...
    ("yaw_idler_spacer",      "base_yaw", "yaw_idler_spacer#1"),   # ... its spacer, ...
    ("yaw_idler_screw",       "base_yaw", "yaw_idler_screw#1"),    # ... its M5 axle down through the plate's slot ...
    ("yaw_idler_nut",         "base_yaw", "yaw_idler_nut#1"),      # ... into its nut, between the rails under the plate
    ("ky003_hall_sensor",     "base_yaw", "ky003_hall_sensor#1"),  # mounted: the base_yaw home sensor under the 120T, ...
    ("yaw_home_bracket",      "base_yaw", "yaw_home_bracket#1"),   # ... on its bracket under the motor plate, ...
    ("yaw_home_screws",       "base_yaw", "yaw_home_screws#1"),    # ... its 2 M3 up ...
    ("yaw_home_nuts",         "base_yaw", "yaw_home_nuts#1"),      # ... into their nuts in the plate's top, ...
    ("yaw_home_board_screws", "base_yaw", "yaw_home_board_screws#1"),   # ... the board's 2 M2 ...
    ("yaw_home_board_nuts",   "base_yaw", "yaw_home_board_nuts#1"),     # ... into the nuts under the bracket
    ("j1_coupler",            None, "j1_coupler#1"),
    ("j1_motor_leg",          None, "j1_motor_leg#1"),     # mounted: the motor-side leg, a ring round the drive's motor sleeve
    ("washer_as6590",         "base_yaw_2", "washer_as6590#2"),    # ... its upper washer under j1_coupler's seat
    ("gt2_pulley_120t",       "base_yaw", "gt2_pulley_120t#1"),    # mounted: the base_yaw 120T on j1_coupler's stub end, in the lower base bearing
    ("yaw_pulley_screws",     "base_yaw", "yaw_pulley_screws#1"),  # mounted: the base_yaw 120T's 4x M4 ...
    ("yaw_pulley_nuts",       "base_yaw", "yaw_pulley_nuts#1"),    # ... into the nuts in j1_coupler's hub
    ("yaw_home_magnet",       "base_yaw", "yaw_home_magnet#1"),    # mounted: the home magnet in the 120T's outer face
    ("fork_hub_screws",       "shoulder_pitch", "fork_hub_screws#1"),      # mounted: the drive's hub to the fork's hub leg, 4x M4 ...
    ("fork_hub_nuts",         "shoulder_pitch", "fork_hub_nuts#1"),        # ... into the nuts captive in the hub
    ("fork_foot_screws",      "shoulder_pitch", "fork_foot_screws#1"),     # mounted: the motor leg's foot to the disc, 2x M4 ...
    ("fork_foot_nuts",        "shoulder_pitch", "fork_foot_nuts#1"),       # ... into the nuts in the disc's slots
    ("fork_sleeve_screws",    "shoulder_pitch", "fork_sleeve_screws#1"),   # mounted: the motor leg's ring to the sleeve, 3x M4 ...
    ("fork_sleeve_nuts",      "shoulder_pitch", "fork_sleeve_nuts#1"),     # ... into the nuts pressed into its lugs
    ("cycloidal_drive",       None, DRIVE_KEY),            # module: assemblies/cycloidal_drive.py (the shoulder_pitch joint)
    ("j1_link",               None, "j1_link#1"),
    ("bearing_6806",          "elbow_pitch_1", "bearing_6806#3"),  # mounted: j1_link's elbow bore
    ("bearing_6806",          "elbow_pitch_2", "bearing_6806#4"),
    ("eg17_g5_gearbox",       "elbow_pitch", "eg17_g5_gearbox#1"),      # mounted: the elbow motor's gearbox on the web across j1_link's motor hole ...
    ("elbow_gearbox_screws",  "elbow_pitch", "elbow_gearbox_screws#1"), # ... its 4x M4, up through the web
    ("nema17_40mm",           "elbow_pitch", "nema17_40mm#2"),     # mounted: on the gearbox's input end ...
    ("mks_servo42d",          "elbow_pitch", "mks_servo42d#2"),
    ("elbow_motor_screws",    "elbow_pitch", "elbow_motor_screws#1"),   # ... its 4x M3, through the gearbox's input plate
    ("gt2_pulley_20t_8mm",    "elbow_pitch", "gt2_pulley_20t_8mm#1"),   # mounted: on the gearbox's shaft, level with the elbow 90T
    ("elbow_belt",            "elbow_pitch", "elbow_belt#1"),      # mounted: the elbow belt, the 20T to the elbow 90T round ...
    ("gt2_idler_20t",         "elbow_pitch", "gt2_idler_20t#1"),   # mounted: ... its tensioner under j1_link's slab, ...
    ("elbow_idler_spacer",    "elbow_pitch", "elbow_idler_spacer#1"),   # ... its spacer, ...
    ("elbow_idler_screw",     "elbow_pitch", "elbow_idler_screw#1"),    # ... its M5 axle up through the slab's floor ...
    ("elbow_idler_nut",       "elbow_pitch", "elbow_idler_nut#1"),      # ... into its nut, in the slot from the top face
    ("gt2_pulley_90t",        "j2", "gt2_pulley_90t#3"),           # mounted: re-seated on the lower elbow bearing (#1 retired)
    ("elbow_pulley_screws",   "elbow_pitch", "elbow_pulley_screws#1"),   # mounted: the elbow 90T's 4x M4 ...
    ("elbow_pulley_nuts",     "elbow_pitch", "elbow_pulley_nuts#1"),     # ... into the block's captive nuts
    # j3_coupler#1 (the elbow coupler) is retired: the roll drive's block bolts to the elbow pulley in its place
    ("forearm_roll_drive",    None, ROLL_KEY),             # module: assemblies/forearm_roll_drive.py (the forearm_roll joint; lib/mounts.py MODULE_MOUNTS)
    ("j2_link",               None, "j2_link#1"),
    ("bearing_6806",          "wrist_pitch_1", "bearing_6806#5"),  # mounted: j2_link's wrist boss
    ("bearing_6806",          "wrist_pitch_2", "bearing_6806#6"),
    ("nema17_40mm",           "wrist_pitch", "nema17_40mm#3"),     # mounted: j2_link's web, on round holes
    ("mks_servo42d",          "wrist_pitch", "mks_servo42d#3"),
    ("gt2_pulley_20t",        "wrist_pitch", "gt2_pulley_20t#4"),  # mounted: on the wrist motor's shaft, level with the wrist 90T
    ("wrist_motor_screws",    "wrist_pitch", "wrist_motor_screws#1"),   # mounted: the wrist motor's 4x M3, up through the web
    ("wrist_belt",            "wrist_pitch", "wrist_belt#1"),      # mounted: the wrist belt, round ...
    ("gt2_idler_20t",         "wrist_pitch", "gt2_idler_20t#3"),   # mounted: ... its tensioner under j2_link's web, ...
    ("wrist_idler_screw",     "wrist_pitch", "wrist_idler_screw#1"),    # ... its M5 axle up through the web's slot ...
    ("wrist_idler_nut",       "wrist_pitch", "wrist_idler_nut#1"),      # ... into its nut, in the channel from the top face
    ("gt2_pulley_90t",        "j3", "gt2_pulley_90t#4"),           # mounted: re-seated on the lower wrist bearing (#2 retired)
    ("wrist_pulley_screws",   "wrist_pitch", "wrist_pulley_screws#1"),   # mounted: the wrist 90T's 4x M4 ...
    ("wrist_pulley_nuts",     "wrist_pitch", "wrist_pulley_nuts#1"),     # ... into the nuts in wrist_link's channels
    # j3_coupler#2 (the wrist coupler) is retired: wrist_link's plate carries the wrist_pitch output flange in its place
    ("wrist_link",            None, "wrist_link#1"),       # the wrist roll drive's frame (its DEFAULT build)
    # nema17_pancake#1 / gt2_pulley_20t#1 are retired and gripper_clamp_bracket skipped: the wrist roll drive carries the
    # pancake (on the pitch axis) and its 20T, the gripper's connector (in the gripper module) bolts to its pulley
    ("wrist_roll_drive",      None, WRIST_ROLL_KEY),       # module: assemblies/wrist_roll_drive.py (the wrist_roll joint; lib/mounts.py MODULE_MOUNTS)
    ("gripper",               None, GRIPPER_KEY),          # module: assemblies/gripper.py (on wrist_link#1 at the connector's plate; lib/mounts.py MODULE_MOUNTS)
]

MODULES = {"gripper": gripper.gripper, "cycloidal_drive": cycloidal_drive.cycloidal_drive,
           "forearm_roll_drive": forearm_roll_drive.forearm_roll_drive,
           "wrist_roll_drive": wrist_roll_drive.wrist_roll_drive}   # the child MODELS

# The component tree: the rigid-link partition of robot/frames.py LINKS with the modules kept whole
# (wrist = wrist_roll_link + jaw_a_link + jaw_b_link; the cycloidal drive under shoulder_link; the forearm roll
# drive under elbow_link and the wrist roll drive under wrist_pitch_link although LINKS puts their rotors a link on).
# Rows are (group label, occurrence keys in document order); tests lock the LINKS mirror.
GROUPS = [
    ("base_link",        ("base#1", "bearing_6806#1", "bearing_6806#2", "washer_as6590#1", "bearing_axk6590#1",
                          "base_motor_mount#1", "base_motor_mount_screws#1", "base_motor_mount_nuts#1",
                          "nema17_48mm#1", "mks_servo42d#1", "yaw_motor_screws#1", "gt2_pulley_20t#3", "yaw_belt#1",
                          "gt2_idler_20t#2", "yaw_idler_spacer#1", "yaw_idler_screw#1", "yaw_idler_nut#1",
                          "ky003_hall_sensor#1", "yaw_home_bracket#1", "yaw_home_screws#1", "yaw_home_nuts#1",
                          "yaw_home_board_screws#1", "yaw_home_board_nuts#1")),
    ("shoulder_link",    ("j1_coupler#1", "j1_motor_leg#1", "washer_as6590#2", "gt2_pulley_120t#1", "yaw_pulley_screws#1",
                          "yaw_pulley_nuts#1", "yaw_home_magnet#1", "fork_hub_screws#1", "fork_hub_nuts#1", "fork_foot_screws#1",
                          "fork_foot_nuts#1", "fork_sleeve_screws#1", "fork_sleeve_nuts#1", DRIVE_KEY)),
    ("upper_arm_link",   ("j1_link#1", "bearing_6806#3", "bearing_6806#4", "eg17_g5_gearbox#1", "elbow_gearbox_screws#1",
                          "nema17_40mm#2", "mks_servo42d#2", "elbow_motor_screws#1", "gt2_pulley_20t_8mm#1", "elbow_belt#1",
                          "gt2_idler_20t#1", "elbow_idler_spacer#1", "elbow_idler_screw#1", "elbow_idler_nut#1")),
    ("elbow_link",       ("gt2_pulley_90t#3", "elbow_pulley_screws#1", "elbow_pulley_nuts#1", ROLL_KEY)),
    ("forearm_link",     ("j2_link#1", "bearing_6806#5", "bearing_6806#6", "nema17_40mm#3", "mks_servo42d#3",
                          "gt2_pulley_20t#4", "wrist_motor_screws#1", "wrist_belt#1", "gt2_idler_20t#3",
                          "wrist_idler_screw#1", "wrist_idler_nut#1")),
    ("wrist_pitch_link", ("gt2_pulley_90t#4", "wrist_pulley_screws#1", "wrist_pulley_nuts#1", "wrist_link#1",
                          WRIST_ROLL_KEY)),
    ("wrist",            (GRIPPER_KEY,)),
]


@step(out="arm.step")
def arm():
    """The arm as a labelled Compound: 'arm' -> the GROUPS component nodes -> parts + the
    modules, every part in its colour (_occurrences.colour(): two tones on the printed
    parts, grey on the purchased ones), in the base_link frame (Z up - arm_from_w())."""
    return assembly("arm", grouped_children(OCCURRENCES, GROUPS, MODULES, root=arm_from_w()))


if __name__ == "__main__":
    arm()   # build: writes the sibling arm.step
