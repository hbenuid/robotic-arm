"""The arm assembly rebuilt from parts + placements reproduces the SolidWorks totals, plus the
code-driven cycloidal_drive module's own totals."""
import pytest
from build123d import Location, Vector

from assemblies import arm, cycloidal_drive, gripper
from lib.models import raw
from lib import placements as P
from lib import reference as R

EXPECTED = P.DATA["expected"]
DRIVE = cycloidal_drive.EXPECTED


def _leaves(node):
    return [node] if not node.children else [leaf for child in node.children for leaf in _leaves(child)]


def _expected_bbox():
    """Union of the world bounding boxes of every part occurrence in placements.json and of the
    designed modules' SolidWorks nodes (the drive's fasteners sit inside that envelope),
    re-expressed in the frame the arm is emitted in (arm.ARM_FROM_W: W, +Y up -> base_link, Z up).
    That map is an axis permutation with signs, so two opposite corners carry the whole box."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    boxes = [(P.OCCURRENCES[k]["world_bbox_min"], P.OCCURRENCES[k]["world_bbox_size"]) for k in P.keys(kind="part")]
    boxes += [(P.OCCURRENCES[k]["solidworks"]["world_bbox_min"], P.OCCURRENCES[k]["solidworks"]["world_bbox_size"])
              for k in P.keys(kind="module", designed=True)]
    for bmin, bsize in boxes:
        for i in range(3):
            lo[i] = min(lo[i], bmin[i])
            hi[i] = max(hi[i], bmin[i] + bsize[i])
    corners = [tuple((arm.ARM_FROM_W * Location(tuple(p))).position) for p in (lo, hi)]
    lo = [min(c[i] for c in corners) for i in range(3)]
    hi = [max(c[i] for c in corners) for i in range(3)]
    return lo, [h - l for h, l in zip(hi, lo)]


@pytest.mark.slow
def test_gripper_module_builds():
    g = raw(gripper.gripper)
    assert g.label == "gripper"
    assert len(_leaves(g)) == len(gripper.OCCURRENCES) == 19
    assert g.is_valid


@pytest.mark.slow
def test_cycloidal_drive_module_builds():
    d = raw(cycloidal_drive.cycloidal_drive)
    assert d.label == "cycloidal_drive"
    assert len(_leaves(d)) == len(cycloidal_drive.OCCURRENCES) == DRIVE["leaves"] == 18
    assert len(d.solids()) == DRIVE["solids"]
    assert abs(R.solid_volume(d) - DRIVE["solid_volume"]) <= 0.5
    assert d.is_valid


def test_arm_groups_mirror_links():
    """GROUPS covers every occurrence key exactly once and mirrors robot/frames.py LINKS with the
    two modules kept whole: wrist = wrist_roll_link + jaw_a_link + jaw_b_link, and the cycloidal
    drive sits under shoulder_link although LINKS puts its rotor body in upper_arm_link."""
    from assemblies._occurrences import split_key
    from robot import frames

    group_keys = [key for _, _, keys in arm.GROUPS for key in keys]
    assert sorted(group_keys) == sorted(key for _, _, key in arm.OCCURRENCES)
    groups = {label: set(keys) for label, _, keys in arm.GROUPS}

    def whole(link):
        return {split_key(k)[0] for k in frames.LINKS[link]}

    assert groups["base_link"] == whole("base_link")
    assert groups["shoulder_link"] == whole("shoulder_link")
    assert groups["upper_arm_link"] == whole("upper_arm_link") - {arm.DRIVE_KEY}
    for link in ("forearm_link", "wrist_pitch_link"):
        assert groups[link] == set(frames.LINKS[link]), link
    wrist = (groups["wrist"] - {arm.GRIPPER_KEY}) | {key for _, _, key in gripper.OCCURRENCES}
    assert wrist == set(
        frames.LINKS["wrist_roll_link"] + frames.LINKS["jaw_a_link"] + frames.LINKS["jaw_b_link"]
    )


def test_arm_is_emitted_z_up():
    """The arm leaves the SolidWorks capture frame W (+Y up) for the base_link frame of
    robot/frames.py (REP-103) - cadgen's viewer and snapshots are Z-up, a W-frame arm.step lies on
    its side: world up -> +Z, the arm's forward -> +X, the base's mounting face centre -> origin."""
    from robot import frames

    def direction(v):
        return (arm.ARM_FROM_W * Location(v)).position - arm.ARM_FROM_W.position

    assert (direction(frames.U) - Vector(0, 0, 1)).length < 1e-9
    assert (direction(frames.BASE_FORWARD) - Vector(1, 0, 0)).length < 1e-9
    assert (arm.ARM_FROM_W * Location((0.0, frames.BASE_BOTTOM_Y, 0.0))).position.length < 1e-9


@pytest.mark.slow
def test_arm_assembly_matches_reference_totals():
    """SolidWorks totals for the SolidWorks-driven occurrences + the drive module's own totals,
    under the GROUPS component tree (arm -> base_link/shoulder_link/upper_arm_link/forearm_link/
    wrist_pitch_link/wrist)."""
    a = raw(arm.arm)
    assert a.label == "arm"
    assert [c.label for c in a.children] == [label for label, _, _ in arm.GROUPS]
    leaves = _leaves(a)
    assert len(leaves) == EXPECTED["leaf_occurrences"] + DRIVE["leaves"] == 52
    labels = [leaf.label for leaf in leaves]
    assert len(set(labels)) == len(labels), f"duplicate leaf labels: {labels}"
    assert len(a.solids()) == EXPECTED["solids"] + DRIVE["solids"] == 108
    assert abs(R.solid_volume(a) - (EXPECTED["solid_volume"] + DRIVE["solid_volume"])) <= 0.5
    exp_min, exp_size = _expected_bbox()
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_min(a), exp_min)), (R.bbox_min(a), exp_min)
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_size(a), exp_size)), (R.bbox_size(a), exp_size)
    assert a.is_valid
