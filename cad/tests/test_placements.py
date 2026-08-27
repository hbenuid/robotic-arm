"""reference/placements.json integrity and its coverage by the assembly tables (no geometry)."""
import math

from build123d import Location

from assemblies import arm, gripper
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
    assert P.keys(kind="module") == ["gripper#1"]
    assert sum(P.OCCURRENCES[k]["solids"] for k in parts_) == P.DATA["expected"]["solids"] == 50


def test_keys_unique_and_parts_known():
    keys = [o["key"] for o in P.DATA["occurrences"]]
    assert len(keys) == len(set(keys))
    for o in P.DATA["occurrences"]:
        assert o["part"] in (set(R.CUSTOM) | set(R.COTS) | set(R.MODULES)), o["key"]
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
    used = [key for _, _, key in arm.OCCURRENCES if key != arm.GRIPPER_KEY] + [key for _, _, key in gripper.OCCURRENCES]
    assert sorted(used) == sorted(P.keys(kind="part"))
    assert [key for _, _, key in arm.OCCURRENCES if key == arm.GRIPPER_KEY] == ["gripper#1"]
    assert all(P.OCCURRENCES[key]["parent"] == "gripper#1" for _, _, key in gripper.OCCURRENCES)
    assert all(P.OCCURRENCES[key]["parent"] is None for _, _, key in arm.OCCURRENCES)
    for name, _, key in arm.OCCURRENCES + gripper.OCCURRENCES:
        assert P.OCCURRENCES[key]["part"] == name, f"{key} is not a {name} occurrence"


def test_duplicate_parts_have_unique_labels():
    for rows in (arm.OCCURRENCES, gripper.OCCURRENCES):
        labels = [name if role is None else f"{name}:{role}" for name, role, _ in rows]
        assert len(labels) == len(set(labels)), f"duplicate labels: {labels}"


def test_skipped_records_the_cycloidal_drive():
    assert any("cyloidal" in s["label"].lower() for s in P.DATA["skipped"])
