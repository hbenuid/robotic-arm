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


@pytest.mark.slow
def test_arm_assembly_matches_reference_totals():
    """SolidWorks totals for the SolidWorks-driven occurrences + the drive module's own totals."""
    a = arm.gen_step()
    assert a.label == "arm"
    assert len(a.children) == len(arm.OCCURRENCES) == 17
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
