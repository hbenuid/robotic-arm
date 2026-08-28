"""The robot description (robot/frames.py, robot/links/*, robot/meshes/*, robot/arm.{urdf,srdf,sdf})
stays consistent with the CAD and with itself."""
import math
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from robot import frames as F

CAD_DIR = pathlib.Path(__file__).resolve().parent.parent
ROBOT_DIR = CAD_DIR / "robot"
URDF, SRDF, SDF = ROBOT_DIR / "arm.urdf", ROBOT_DIR / "arm.srdf", ROBOT_DIR / "arm.sdf"
PHYSICAL_LINKS = [l for l in F.LINK_ORDER if F.LINKS[l]]

sys.path.insert(0, str(CAD_DIR / "tools"))
import robot_frames as RF  # noqa: E402


# --- fast: structure ---------------------------------------------------------------------------
def test_links_partition_every_placement_once():
    keys = F.all_keys()
    assert len(keys) == len(set(keys)), "a placement key is in two links"
    assert sorted(keys) == sorted(P.keys(kind="part") + P.keys(kind="module", designed=True))
    assert "cycloidal_drive#1" in F.LINKS["link1"]


def test_joint_tree_is_a_tree_rooted_at_base_link():
    children = {j.child for j in F.JOINTS}
    assert len(children) == len(F.JOINTS), "a link has two parent joints"
    assert "base_link" not in children
    for j in F.JOINTS:
        assert j.parent in F.LINK_ORDER and j.child in F.LINK_ORDER
    assert set(F.LINK_ORDER) == children | {"base_link"}


def test_joint_frames_are_orthonormal_with_z_on_the_axis():
    for j in F.JOINTS:
        M = RF.matrix(F.joint_frame_world(j.name))
        cols = [[M[i][c] for i in range(3)] for c in range(3)]
        for a in range(3):
            assert math.isclose(sum(v * v for v in cols[a]), 1.0, abs_tol=1e-9)
            for b in range(a + 1, 3):
                assert abs(sum(x * y for x, y in zip(cols[a], cols[b]))) < 1e-9
        assert all(abs(z - a) < 1e-5 for z, a in zip(cols[2], j.axis_w)), j.name
        # right-handed: x cross y == z
        x, y, z = cols
        cross = (x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0])
        assert all(abs(c - v) < 1e-9 for c, v in zip(cross, z)), j.name


def test_rpy_round_trip():
    for j in F.JOINTS:
        RF.joint_origin(j)   # asserts internally that R(rpy) reproduces the matrix


def test_meshes_exist_and_are_referenced_by_the_urdf():
    root = ET.parse(URDF).getroot()
    for link in F.LINK_ORDER:
        el = root.find(f"link[@name='{link}']")
        assert el is not None, link
        if F.LINKS[link]:
            mesh = ROBOT_DIR / "meshes" / f"{link}.stl"
            assert mesh.exists() and mesh.stat().st_size > 84 * 2, mesh
            for tag in ("visual", "collision"):
                m = el.find(f"{tag}/geometry/mesh")
                assert m is not None and m.get("filename") == f"meshes/{link}.stl"
                assert m.get("scale") == "0.001 0.001 0.001"
        else:
            assert el.find("visual") is None and el.find("inertial") is None, f"{link} is frame-only"


def test_urdf_srdf_sdf_are_consistent():
    urdf = ET.parse(URDF).getroot()
    srdf = ET.parse(SRDF).getroot()
    sdf = ET.parse(SDF).getroot().find("model")
    assert urdf.get("name") == srdf.get("name") == sdf.get("name") == F.ROBOT_NAME
    ulinks = {l.get("name") for l in urdf.findall("link")}
    ujoints = {j.get("name") for j in urdf.findall("joint")}
    assert ulinks == set(F.LINK_ORDER) and ujoints == {j.name for j in F.JOINTS}
    assert {l.get("name") for l in sdf.findall("link")} == ulinks
    assert {j.get("name") for j in sdf.findall("joint")} == ujoints
    for el in srdf.iter():
        if el.tag in ("joint", "passive_joint") and el.get("name"):
            assert el.get("name") in ujoints, el.get("name")
        if el.tag == "link":
            assert el.get("name") in ulinks
        if el.tag == "disable_collisions":
            assert el.get("link1") in ulinks and el.get("link2") in ulinks
    chain = srdf.find("group[@name='arm']/chain")
    assert chain.get("base_link") == "base_link" and chain.get("tip_link") == "tool0"


def test_urdf_limits_track_params():
    root = ET.parse(URDF).getroot()
    lim = root.find("joint[@name='j1']/limit")
    assert math.isclose(float(lim.get("upper")), math.radians(PARAMS.J1_LIMIT_DEG), abs_tol=1e-6)
    lim = root.find("joint[@name='jaw_a']/limit")
    assert math.isclose(float(lim.get("upper")), PARAMS.JAW_TRAVEL_MM * 1e-3, abs_tol=1e-9)
    mimic = root.find("joint[@name='jaw_b']/mimic")
    assert mimic.get("joint") == "jaw_a" and float(mimic.get("multiplier")) == -1.0


def _hom(xyz, rpy):
    R = RF.matrix_from_rpy(*rpy)
    return [[*R[i], xyz[i]] for i in range(3)] + [[0, 0, 0, 1]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def test_forward_kinematics_at_zero_reproduces_the_capture_frames():
    """Composing the URDF joint origins (all joints 0) from base_link must land on every link's
    world frame in the SolidWorks capture - the 'zero pose = capture pose' contract."""
    root = ET.parse(URDF).getroot()
    origins = {j.get("name"): (RF._floats(j.find("origin").get("xyz")), RF._floats(j.find("origin").get("rpy")))
               for j in root.findall("joint")}
    Mb = RF.matrix(F.BASE_FRAME)
    world = {"base_link": [[*Mb[i][:3], Mb[i][3] * 1e-3] for i in range(3)] + [[0, 0, 0, 1]]}
    for j in F.JOINTS:  # JOINTS is in tree order (parents first)
        xyz, rpy = origins[j.name]
        world[j.child] = _mul(world[j.parent], _hom(xyz, rpy))
    for link in F.LINK_ORDER:
        M = RF.matrix(F.link_frame_world(link))
        want = [[*M[i][:3], M[i][3] * 1e-3] for i in range(3)]
        got = world[link]
        for i in range(3):
            for k in range(4):
                assert abs(got[i][k] - want[i][k]) < 1e-6, f"{link} [{i}][{k}]: {got[i][k]} vs {want[i][k]}"


# --- slow: geometry ----------------------------------------------------------------------------
@pytest.mark.slow
@pytest.mark.parametrize("link", PHYSICAL_LINKS)
def test_link_builds_from_its_occurrences(link):
    from robot._links import build_link

    from assemblies import cycloidal_drive

    shape = build_link(link)
    assert shape.label == link
    solids = volume = 0.0
    for k in F.LINKS[link]:
        o = P.OCCURRENCES[k]
        if o.get("designed"):     # a code-driven module: its own totals lock
            assert o["part"] == "cycloidal_drive"
            solids += cycloidal_drive.EXPECTED["solids"]
            volume += cycloidal_drive.EXPECTED["solid_volume"]
        else:
            solids += o["solids"]
            volume += o["solid_volume"]
    assert len(shape.solids()) == solids
    assert abs(R.solid_volume(shape) - volume) <= 0.5
    assert shape.is_valid


@pytest.mark.slow
def test_link_masses_add_up():
    """Every part of every link counted once: SolidWorks part keys from placements.json, the
    designed module's parts from a fresh build."""
    import importlib

    from assemblies._occurrences import world_rows

    expected_g = 0.0
    for k in P.keys(kind="part") + P.keys(kind="module", designed=True):
        for part, _, _ in world_rows(k):
            mod = importlib.import_module(f"parts.{part}")
            if part in R.COTS:
                expected_g += mod.MASS_G
            elif P.OCCURRENCES[k]["kind"] == "part":
                expected_g += PARAMS.PETG_DENSITY * P.OCCURRENCES[k]["solid_volume"]
            else:
                expected_g += PARAMS.PETG_DENSITY * R.solid_volume(mod.gen_step())
    total = sum(RF.link_inertial(l)[0] for l in PHYSICAL_LINKS)
    assert math.isclose(total, expected_g * 1e-3, rel_tol=1e-6)


@pytest.mark.slow
def test_urdf_and_sdf_match_the_frames_and_cad():
    assert RF.check_urdf(URDF) == []
    assert RF.check_sdf(SDF) == []


@pytest.mark.slow
@pytest.mark.parametrize("target", ["robot/arm.urdf", "robot/arm.srdf", "robot/arm.sdf"])
def test_plugin_validators_pass(target):
    """The text-to-cad urdf/srdf/sdf validators (strict) accept the checked-in files."""
    args = ["./cadtool", "validate", target] + (["--strict"] if not target.endswith(".sdf") else ["--gz-check", "never"])
    proc = subprocess.run(args, cwd=CAD_DIR, capture_output=True, text=True)
    if "plugin not found" in proc.stderr:
        pytest.skip("cad@text-to-cad plugin not installed")
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
