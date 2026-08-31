"""The arm assembly rebuilt from parts + placements reproduces the SolidWorks totals, plus the
code-driven cycloidal_drive module's own totals."""
import pytest

from assemblies import arm, cycloidal_drive, gripper
from lib import placements as P
from lib import reference as R

EXPECTED = P.DATA["expected"]
DRIVE = cycloidal_drive.EXPECTED


def _leaves(node):
    return [node] if not node.children else [leaf for child in node.children for leaf in _leaves(child)]


def _expected_bbox():
    """Union of the world bounding boxes of every part occurrence in placements.json and of the
    designed modules' SolidWorks nodes (the drive's fasteners sit inside that envelope)."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    boxes = [(P.OCCURRENCES[k]["world_bbox_min"], P.OCCURRENCES[k]["world_bbox_size"]) for k in P.keys(kind="part")]
    boxes += [(P.OCCURRENCES[k]["solidworks"]["world_bbox_min"], P.OCCURRENCES[k]["solidworks"]["world_bbox_size"])
              for k in P.keys(kind="module", designed=True)]
    for bmin, bsize in boxes:
        for i in range(3):
            lo[i] = min(lo[i], bmin[i])
            hi[i] = max(hi[i], bmin[i] + bsize[i])
    return lo, [h - l for h, l in zip(hi, lo)]


@pytest.mark.slow
def test_gripper_module_builds():
    g = gripper.gen_step()
    assert g.label == "gripper"
    assert len(_leaves(g)) == len(gripper.OCCURRENCES) == 19
    assert g.is_valid


@pytest.mark.slow
def test_cycloidal_drive_module_builds():
    d = cycloidal_drive.gen_step()
    assert d.label == "cycloidal_drive"
    assert len(_leaves(d)) == len(cycloidal_drive.OCCURRENCES) == DRIVE["leaves"] == 18
    assert len(d.solids()) == DRIVE["solids"]
    assert abs(R.solid_volume(d) - DRIVE["solid_volume"]) <= 0.5
    assert d.is_valid


def test_arm_groups_mirror_links():
    """GROUPS covers every occurrence key exactly once and mirrors robot/frames.py LINKS
    (gripper module kept whole: wrist = wrist_roll_link + jaw_a_link + jaw_b_link)."""
    from robot import frames

    group_keys = [key for _, _, keys in arm.GROUPS for key in keys]
    assert sorted(group_keys) == sorted(key for _, _, key in arm.OCCURRENCES)
    groups = {label: set(keys) for label, _, keys in arm.GROUPS}
    for link in ("base_link", "link1", "link2", "link3"):
        assert groups[link] == set(frames.LINKS[link]), link
    wrist = (groups["wrist"] - {arm.GRIPPER_KEY}) | {key for _, _, key in gripper.OCCURRENCES}
    assert wrist == set(
        frames.LINKS["wrist_roll_link"] + frames.LINKS["jaw_a_link"] + frames.LINKS["jaw_b_link"]
    )


@pytest.mark.slow
def test_arm_assembly_matches_reference_totals():
    """SolidWorks totals for the SolidWorks-driven occurrences + the drive module's own totals,
    under the GROUPS component tree (arm -> base_link/link1/link2/link3/wrist)."""
    a = arm.gen_step()
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
