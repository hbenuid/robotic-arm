"""reference/placements.json integrity and its coverage by the assembly tables (no geometry)."""
import math

from build123d import Location

from assemblies import arm, cycloidal_drive, gripper
from lib import placements as P
from lib import reference as R


def _matrix(loc: Location):
    t = loc.wrapped.Transformation()
    return [[t.Value(i, j) for j in (1, 2, 3, 4)] for i in (1, 2, 3)]


def _close(a, b, tol=1e-6):
    return all(math.isclose(x, y, abs_tol=tol) for ra, rb in zip(a, b) for x, y in zip(ra, rb))


def test_record_counts_match_expected():
    parts_ = P.keys(kind="part")
    assert len(parts_) == P.DATA["expected"]["leaf_occurrences"] == 34
    assert P.keys(kind="module") == ["cycloidal_drive#1", "gripper#1"]
    assert P.keys(kind="module", designed=True) == P.DATA["designed_modules"] == ["cycloidal_drive#1"]
    assert sum(P.OCCURRENCES[k]["solids"] for k in parts_) == P.DATA["expected"]["solids"] == 50


def test_keys_unique_and_parts_known():
    keys = [o["key"] for o in P.DATA["occurrences"]]
    assert len(keys) == len(set(keys))
    for o in P.DATA["occurrences"]:
        assert o["part"] in (set(R.CUSTOM) | set(R.COTS) | set(R.MODULES) | set(R.DESIGNED_MODULES)), o["key"]
        assert o["key"] == f"{o['part']}#{o['key'].rsplit('#', 1)[1]}"
        if o["parent"] is not None:
            assert o["parent"] in P.OCCURRENCES and P.OCCURRENCES[o["parent"]]["kind"] == "module"


def test_rotation_convention_round_trips():
    """Location(position, rotation_xyz_deg) must reproduce the stored 3x4 matrix."""
    for o in P.DATA["occurrences"]:
        for frame in ("rel", "world"):
            assert _close(_matrix(P.to_location(o[frame])), o[frame]["matrix_3x4"]), f"{o['key']} {frame}"


def test_world_equals_parent_world_times_rel():
    for key, o in P.OCCURRENCES.items():
        parent_world = P.location(o["parent"], "world") if o["parent"] else Location()
        # 1e-4 mm/unitless: the JSON stores 6-decimal positions/angles, and a ~1e-6 deg
        # rounding over a ~100 mm lever arm shows up at the 1e-6 level.
        assert _close(_matrix(parent_world * P.location(key, "rel")), _matrix(P.location(key, "world")), tol=1e-4), key


def test_assembly_tables_claim_every_part_key_exactly_once():
    used = [key for _, _, key in arm.OCCURRENCES if key not in arm.MODULE_KEYS] + [key for _, _, key in gripper.OCCURRENCES]
    assert sorted(used) == sorted(P.keys(kind="part"))
    assert [key for _, _, key in arm.OCCURRENCES if key in arm.MODULE_KEYS] == ["cycloidal_drive#1", "gripper#1"]
    assert all(P.OCCURRENCES[key]["parent"] == "gripper#1" for _, _, key in gripper.OCCURRENCES)
    assert all(P.OCCURRENCES[key]["parent"] is None for _, _, key in arm.OCCURRENCES)
    for name, _, key in arm.OCCURRENCES + gripper.OCCURRENCES:
        assert P.OCCURRENCES[key]["part"] == name, f"{key} is not a {name} occurrence"


def test_duplicate_parts_have_unique_labels():
    for rows in (arm.OCCURRENCES, gripper.OCCURRENCES, cycloidal_drive.OCCURRENCES):
        labels = [name if role is None else f"{name}:{role}" for name, role, _ in rows]
        assert len(labels) == len(set(labels)), f"duplicate labels: {labels}"


def test_designed_module_records_the_cycloidal_drive():
    """The drive's SolidWorks node is a designed module: pose from SolidWorks (verbatim the
    former skipped[0]), contents from assemblies/cycloidal_drive.py; nothing is skipped any more."""
    o = P.OCCURRENCES["cycloidal_drive#1"]
    assert o["kind"] == "module" and o["designed"] is True and o["parent"] is None and o["path"] == "1.3"
    assert o["part"] == "cycloidal_drive" and o["label_in_monolith"] == "New_cyloidal_assembly"
    assert o["source"] == "assemblies/cycloidal_drive.py"
    assert o["world"]["position"] == [1.844381, 85.010435, 31.446506]
    assert o["world"]["rotation_xyz_deg"] == [-180.0, -3.694455, 180.0]
    assert o["rel"] == o["world"]
    sw = o["solidworks"]
    assert (sw["leaves"], sw["solids"], sw["solid_volume"]) == (15, 38, 674390.543)
    assert sw["world_bbox_min"] == [-71.78, 15.01, -35.683] and sw["world_bbox_size"] == [143.382, 140.0, 116.393]
    assert "solids" not in o and "solid_volume" not in o     # totals come from the module build, not SolidWorks
    assert P.DATA["skipped"] == []
