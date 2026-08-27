"""The arm assembly rebuilt from parts + placements reproduces the SolidWorks totals."""
import pytest

from assemblies import arm, gripper
from lib import placements as P
from lib import reference as R

EXPECTED = P.DATA["expected"]


def _leaves(node):
    return [node] if not node.children else [leaf for child in node.children for leaf in _leaves(child)]


def _expected_bbox():
    """Union of the world bounding boxes of every part occurrence in placements.json."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for key in P.keys(kind="part"):
        o = P.OCCURRENCES[key]
        for i in range(3):
            lo[i] = min(lo[i], o["world_bbox_min"][i])
            hi[i] = max(hi[i], o["world_bbox_min"][i] + o["world_bbox_size"][i])
    return lo, [h - l for h, l in zip(hi, lo)]


@pytest.mark.slow
def test_gripper_module_builds():
    g = gripper.gen_step()
    assert g.label == "gripper"
    assert len(_leaves(g)) == len(gripper.OCCURRENCES) == 19
    assert g.is_valid


@pytest.mark.slow
def test_arm_assembly_matches_reference_totals():
    a = arm.gen_step()
    assert a.label == "arm"
    assert len(a.children) == len(arm.OCCURRENCES)
    leaves = _leaves(a)
    assert len(leaves) == EXPECTED["leaf_occurrences"]
    labels = [leaf.label for leaf in leaves]
    assert len(set(labels)) == len(labels), f"duplicate leaf labels: {labels}"
    assert len(a.solids()) == EXPECTED["solids"]
    assert abs(R.solid_volume(a) - EXPECTED["solid_volume"]) <= 0.5
    exp_min, exp_size = _expected_bbox()
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_min(a), exp_min)), (R.bbox_min(a), exp_min)
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_size(a), exp_size)), (R.bbox_size(a), exp_size)
    assert a.is_valid
