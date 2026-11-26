"""robotic_arm.config: the joint table's shape. Its numbers against the CAD: cad/tests/test_robot.py."""

from __future__ import annotations

import ast
import pathlib

from robotic_arm import config

URDF_CHAIN = ("base_yaw", "shoulder_pitch", "elbow_pitch", "forearm_roll", "wrist_pitch", "wrist_roll")


def test_the_table_is_the_urdfs_six_revolute_joints_in_chain_order():
    assert tuple(j.name for j in config.JOINTS) == URDF_CHAIN


def test_each_joint_is_well_formed():
    ids = [j.can_id for j in config.JOINTS if j.can_id is not None]
    assert len(ids) == len(set(ids)) and all(1 <= i <= 0x7FF for i in ids)
    for j in config.JOINTS:
        assert 0 < j.gear_ratio < 1, f"{j.name}: gear_ratio reads output turns per motor turn, a reduction's reciprocal"
        assert j.sign in (-1, 1)
        lo, hi = j.limits_deg
        assert lo < j.home_deg < hi


def test_config_imports_only_the_standard_library():
    """cad/tests/test_robot.py loads this file by its path, without the package or its dependencies."""
    tree = ast.parse(pathlib.Path(config.__file__).read_text())
    modules = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    modules |= {n.module or "." for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}   # ".": a relative import
    assert modules <= {"__future__", "dataclasses"}
