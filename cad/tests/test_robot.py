"""The robot description (robot/frames.py, robot/links/*, robot/meshes/*, robot/arm.{urdf,srdf,sdf})
stays consistent with the CAD and with itself."""
import functools
import importlib
import math
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import parts
from assemblies._occurrences import placement_at
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from robot import frames as F
from tests import built
from tests.source_checks import runs_its_model
from tests.totals import solids_and_volume
from tools.robot import derive as RF

CAD_DIR = pathlib.Path(__file__).resolve().parent.parent
ROBOT_DIR = CAD_DIR / "robot"
URDF, SRDF, SDF = ROBOT_DIR / "arm.urdf", ROBOT_DIR / "arm.srdf", ROBOT_DIR / "arm.sdf"
PHYSICAL_LINKS = [l for l in F.LINK_ORDER if F.LINKS[l]]
_link_inertial = functools.cache(RF.link_inertial)


@pytest.fixture
def inertials_from_shared_parts(monkeypatch):
    """tools/robot/derive.py's link inertials over tests/built.py's parts (it only measures them), each link's computed
    once per process - two tests read them all."""
    monkeypatch.setattr(RF, "place_world_at",
                        lambda part, world, into=None: built.part(part).moved(placement_at(part, world, into)))
    monkeypatch.setattr(RF, "link_inertial", _link_inertial)


# --- fast: structure ---------------------------------------------------------------------------
def test_every_physical_link_has_a_runnable_model():
    """robot/links/ holds exactly one model file per physical link, and each ends with the
    `__main__` call of its model - without it `./cadtool gen robot/links/<link>.py` builds nothing."""
    files = {p.stem: p for p in (ROBOT_DIR / "links").glob("*.py") if p.stem != "__init__"}
    assert sorted(files) == sorted(PHYSICAL_LINKS)
    for link, path in files.items():
        assert f"@step\ndef {link}():" in path.read_text(encoding="utf-8"), f"{path.name}: expected `@step def {link}()`"
        assert runs_its_model(path, link), f"{path.name}: must end with `if __name__ == \"__main__\": {link}()`"



def test_links_partition_every_placement_once():
    from assemblies._occurrences import module_bodies, split_key

    keys = F.all_keys()
    assert len(keys) == len(set(keys)), "a placement key is in two links"
    whole = sorted({split_key(k)[0] for k in keys})
    assert whole == sorted(P.keys(kind="part") + P.keys(kind="module", designed=True))
    for mkey in P.keys(kind="module", designed=True):    # a designed module: whole once, or every body once
        used = [split_key(k)[1] for k in keys if split_key(k)[0] == mkey]
        bodies = module_bodies(P.OCCURRENCES[mkey]["part"])
        assert used == [None] or (None not in used and sorted(used) == sorted(bodies)), (mkey, used)
    assert "cycloidal_drive#1:stator" in F.LINKS["shoulder_link"]   # housing + motor ride with the holder
    assert "cycloidal_drive#1:rotor" in F.LINKS["upper_arm_link"]    # output hub + pins ride with j1_link
    assert "forearm_roll_drive#1:stator" in F.LINKS["elbow_link"]    # the elbow block rides with the elbow pulley
    assert "forearm_roll_drive#1:rotor" in F.LINKS["forearm_link"]   # the roll shaft IS the forearm's elbow end


def test_world_rows_expands_a_designed_module_whole_or_per_body():
    from assemblies import cycloidal_drive
    from assemblies._occurrences import world_rows

    rows = {body: world_rows(f"cycloidal_drive#1:{body}") for body in cycloidal_drive.BODIES}
    whole = world_rows("cycloidal_drive#1")
    assert len(whole) == len(cycloidal_drive.OCCURRENCES) == sum(len(r) for r in rows.values())
    assert {p for p, _, _ in rows["rotor"]} == cycloidal_drive.ROTOR
    assert {r[:2] for r in whole} == {r[:2] for body_rows in rows.values() for r in body_rows}
    for bad in ("base#1:rotor", "gripper#1:rotor", "cycloidal_drive#1:nope"):
        with pytest.raises(ValueError):
            world_rows(bad)


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
                assert abs(sum(x * y for x, y in zip(cols[a], cols[b], strict=True))) < 1e-9
        assert all(abs(z - a) < 1e-5 for z, a in zip(cols[2], j.axis_w, strict=True)), j.name
        # right-handed: x cross y == z
        x, y, z = cols
        cross = (x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0])
        assert all(abs(c - v) < 1e-9 for c, v in zip(cross, z, strict=True)), j.name


def test_rpy_round_trip():
    for j in F.JOINTS:
        RF.joint_origin(j)   # asserts internally that R(rpy) reproduces the matrix


def test_joint_origins_sit_on_their_shifted_records():
    """The origins beyond a link follow its SHIFTS (lib/placements.py): each is its record's world origin."""
    for origin, key in ((F.ELBOW_ORIGIN, "j2_link#1"), (F.WRIST_PITCH_ORIGIN, "j3_coupler#2"),
                        (F.WRIST_ROLL_ORIGIN, "gt2_pulley_20t#1")):
        assert math.dist(origin, tuple(P.location(key, "world").position)) < 0.01, key


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
    lim = root.find("joint[@name='base_yaw']/limit")
    assert math.isclose(float(lim.get("upper")), math.radians(PARAMS.BASE_YAW_LIMIT_DEG), abs_tol=1e-6)
    lim = root.find("joint[@name='shoulder_pitch']/limit")
    assert math.isclose(float(lim.get("upper")), math.radians(PARAMS.SHOULDER_PITCH_LIMIT_DEG), abs_tol=1e-6)
    lim = root.find("joint[@name='forearm_roll']/limit")
    assert math.isclose(float(lim.get("upper")), math.radians(PARAMS.FOREARM_ROLL_LIMIT_DEG), abs_tol=1e-6)
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
    Mb = RF.matrix(F.base_frame())
    world = {"base_link": [[*Mb[i][:3], Mb[i][3] * 1e-3] for i in range(3)] + [[0, 0, 0, 1]]}
    for j in F.JOINTS:  # JOINTS is in tree order (parents first)
        xyz, rpy = origins[j.name]
        world[j.child] = _mul(world[j.parent], _hom(xyz, rpy))
    # The URDF carries six decimals (1 um / 1 urad per joint origin); composing the five joints
    # down to the wrist legitimately accumulates a few um, so this is a per-chain tolerance
    # (tools/robot/derive.py --check holds every joint to 1e-6 individually).
    tol = 5e-6
    for link in F.LINK_ORDER:
        M = RF.matrix(F.link_frame_world(link))
        want = [[*M[i][:3], M[i][3] * 1e-3] for i in range(3)]
        got = world[link]
        for i in range(3):
            for k in range(4):
                assert abs(got[i][k] - want[i][k]) < tol, f"{link} [{i}][{k}]: {got[i][k]} vs {want[i][k]}"


# --- slow: geometry ----------------------------------------------------------------------------
def _stl_facts(path: pathlib.Path) -> dict:
    """Enclosed volume (mm^3), its centroid and the vertex bbox of a binary STL."""
    data = path.read_bytes()
    n = int(np.frombuffer(data, "<u4", count=1, offset=80)[0])
    assert len(data) == 84 + 50 * n, f"{path.name}: not a binary STL (an LFS pointer? git lfs pull)"
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    tris = np.frombuffer(data, record, count=n, offset=84)["v"].astype(np.float64)
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    vol = np.einsum("ij,ij->i", v0, np.cross(v1, v2)) / 6.0
    pts = tris.reshape(-1, 3)
    return {"volume": vol.sum(), "centroid": ((v0 + v1 + v2) / 4.0 * vol[:, None]).sum(axis=0) / vol.sum(),
            "bbox": np.concatenate([pts.min(axis=0), pts.max(axis=0)])}


@pytest.mark.slow
@pytest.mark.parametrize("link", PHYSICAL_LINKS)
def test_link_builds_from_its_occurrences_and_matches_its_mesh(link, tmp_path):
    """The link as tools/robot/export_link_meshes.py builds and exports it (robot/_links.py build_link): its solids and
    volume are what its occurrences add up to, and robot/meshes/<link>.stl is what the tool writes for it today, so a
    geometry change that skipped the re-export (Recipe C step 8) fails here. The meshes are compared by what they
    enclose, not byte for byte: OCCT triangulates the same geometry a little differently per machine - measured
    against the committed meshes, at most 4.2e-5 relative volume and 0.0033 mm centroid (shoulder_link), bbox
    identical."""
    from assemblies._occurrences import split_key
    from tools.robot.export_link_meshes import export_link

    fresh = tmp_path / f"{link}.stl"
    shape, ok = export_link(link, fresh)
    assert ok and shape.label == link
    solids = volume = 0.0
    for k in F.LINKS[link]:
        okey, body = split_key(k)
        o = P.OCCURRENCES[okey]
        if o.get("designed"):     # a code-driven module: its own totals lock (whole or one rigid body)
            expected = importlib.import_module(f"assemblies.{o['part']}").EXPECTED
            lock = expected["bodies"][body] if body else expected
            solids += lock["solids"]
            volume += lock["solid_volume"]
        else:                     # a SolidWorks record, or the part's own build once converted (tests.totals)
            s, v = solids_and_volume(okey)
            solids += s
            volume += v
    assert len(shape.solids()) == solids
    assert abs(R.solid_volume(shape) - volume) <= 0.5
    now, committed = _stl_facts(fresh), _stl_facts(ROBOT_DIR / "meshes" / f"{link}.stl")
    stale = f"robot/meshes/{link}.stl is stale: ./cadtool python tools/robot/export_link_meshes.py --links {link}"
    assert abs(now["volume"] / committed["volume"] - 1.0) <= 5e-4, stale
    assert np.linalg.norm(now["centroid"] - committed["centroid"]) <= 0.02, stale
    assert np.abs(now["bbox"] - committed["bbox"]).max() <= 0.05, stale


@pytest.mark.slow
def test_link_masses_add_up(inertials_from_shared_parts):
    """Every part of every link counted once: SolidWorks part keys from placements.json, the
    designed module's parts from a fresh build."""
    from assemblies._occurrences import world_rows

    expected_g = 0.0
    for k in P.keys(kind="part") + P.keys(kind="module", designed=True):
        for part, _, _ in world_rows(k):
            mod = parts.load(part)
            if part in R.COTS:
                expected_g += mod.MASS_G
            elif P.OCCURRENCES[k]["kind"] == "part":
                expected_g += PARAMS.PETG_DENSITY * solids_and_volume(k)[1]
            else:
                expected_g += PARAMS.PETG_DENSITY * R.solid_volume(built.part(part))
    total = sum(RF.link_inertial(l)[0] for l in PHYSICAL_LINKS)
    assert math.isclose(total, expected_g * 1e-3, rel_tol=1e-6)


@pytest.mark.slow
def test_urdf_and_sdf_match_the_frames_and_cad(inertials_from_shared_parts):
    assert RF.check_urdf(URDF) == []
    assert RF.check_sdf(SDF) == []


@pytest.mark.slow
@pytest.mark.parametrize("target", ["robot/arm.urdf", "robot/arm.srdf", "robot/arm.sdf"])
def test_plugin_validators_pass(target):
    """cadgen's urdf/srdf/sdf validators (strict) accept the checked-in files."""
    args = ["./cadtool", "validate", target] + (["--strict"] if not target.endswith(".sdf") else ["--gz-check", "never"])
    proc = subprocess.run(args, cwd=CAD_DIR, capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
