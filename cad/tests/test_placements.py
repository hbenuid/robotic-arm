"""reference/placements.json integrity and its coverage by the assembly tables, and the design's link-length
shifts of its records (lib/placements.py SHIFTS) (no geometry)."""
import math
from dataclasses import replace

from build123d import Location

import lib.forearm.params as FOREARM
import lib.upper_arm.params as UPPER_ARM
import lib.wrist.params as WRIST
from assemblies import arm, cycloidal_drive, forearm_roll_drive, gripper
from lib import placements as P
from lib import reference as R
from robot import frames as RF


def _matrix(loc: Location):
    t = loc.wrapped.Transformation()
    return [[t.Value(i, j) for j in (1, 2, 3, 4)] for i in (1, 2, 3)]


def _close(a, b, tol=1e-6):
    return all(math.isclose(x, y, abs_tol=tol) for ra, rb in zip(a, b, strict=True) for x, y in zip(ra, rb, strict=True))


def test_record_counts_match_expected():
    parts_ = P.keys(kind="part", retired=True)
    assert len(parts_) == P.DATA["expected"]["leaf_occurrences"] == 55      # 31 SolidWorks + 24 mounted (j3_coupler#1, the two 90Ts retired, still records)
    assert P.keys(kind="module") == ["cycloidal_drive#1", "gripper#1", "forearm_roll_drive#1"]
    assert P.keys(kind="module", designed=True) == P.DATA["designed_modules"] == ["cycloidal_drive#1", "forearm_roll_drive#1"]
    assert sum(P.OCCURRENCES[k]["solids"] for k in parts_) == P.DATA["expected"]["solids"] == 133  # 47 + the base motor mount (1 + 4 + 4) + (7 + 13) + 2 x (2 + 13) + 6 bearings + 3 thrust + 2 pulleys + 4 x 4 pulley bolts
    assert len(P.keys(kind="part", mounted=False, retired=True)) == 31
    assert P.RETIRED == ("j3_coupler#1", "gt2_pulley_90t#1", "gt2_pulley_90t#2") and set(P.RETIRED) <= set(P.OCCURRENCES)
    assert len(P.keys(kind="part")) == 55 - len(P.RETIRED)


def test_mounted_records_follow_lib_mounts():
    """The mounts (lib/mounts.py: the base's motor mount + its bolts, motors + boards, bearings, re-seated pulleys, their
    bolts) are part records written by
    tools/reference/mount_placements.py:
    parent None, rel == world = host world * the declared frame, the `mount` block naming the declaration."""
    from lib import mounts
    from lib.datum import to_location

    keys = P.keys(kind="part", mounted=True)
    assert keys == mounts.keys() == [
        "base_motor_mount#1", "base_motor_mount_screws#1", "base_motor_mount_nuts#1",
        "nema17_48mm#1", "mks_servo42d#1", "nema17_40mm#2", "mks_servo42d#2", "nema17_40mm#3", "mks_servo42d#3",
        *(f"bearing_6806#{n}" for n in range(1, 7)), "washer_as6590#1", "bearing_axk6590#1", "washer_as6590#2",
        "gt2_pulley_90t#3", "gt2_pulley_90t#4",
        "elbow_pulley_screws#1", "elbow_pulley_nuts#1", "wrist_pulley_screws#1", "wrist_pulley_nuts#1"]
    assert P.keys(mounted=True) == P.DATA["mounted"] == mounts.keys() + mounts.module_keys()
    for key in keys:
        o, m = P.OCCURRENCES[key], mounts.BY_KEY[key]
        assert (o["kind"], o["parent"], o["path"], o["part"]) == ("part", None, None, m.part)
        assert (o["mount"]["host"], o["mount"]["link"], o["mount"]["joint"], o["mount"]["source"]) == (m.host, m.link, m.joint, "lib/mounts.py")
        assert o["mount"]["frame_in_host"] == {"position": list(m.frame[0]), "rotation_xyz_deg": list(m.frame[1])}
        assert o["rel"] == o["world"]
        host_world = P.location(m.host, "world")           # a SolidWorks record, or a mount declared before what it hosts
        assert _close(_matrix(host_world * to_location(m.frame)), _matrix(P.location(key, "world")), tol=1e-4), key
        assert o["solids"] > 0 and o["solid_volume"] > 0 and len(o["world_bbox_min"]) == 3


def test_mounted_module_record_follows_lib_mounts():
    """A designed module the capture never placed (lib/mounts.py MODULE_MOUNTS): a module record with a `mount`
    block, listed under designed_modules AND mounted, rel == world = host world * the declared frame, no totals
    (its own EXPECTED is the lock), and its +Z on its joint's axis."""
    from lib import mounts
    from lib.datum import to_location
    from robot import frames as F

    assert mounts.module_keys() == ["forearm_roll_drive#1"]
    for key in mounts.module_keys():
        o, m = P.OCCURRENCES[key], mounts.MODULES_BY_KEY[key]
        assert (o["kind"], o["designed"], o["parent"], o["path"], o["part"], o["source"]) == ("module", True, None, None, m.module, f"assemblies/{m.module}.py")
        assert (o["mount"]["host"], o["mount"]["joint"], o["mount"]["source"]) == (m.host, m.joint, "lib/mounts.py") and "link" not in o["mount"]
        assert o["mount"]["frame_in_host"] == {"position": list(m.frame[0]), "rotation_xyz_deg": list(m.frame[1])}
        assert o["rel"] == o["world"] and "solids" not in o and "solid_volume" not in o
        world = P.location(key, "world")
        assert _close(_matrix(P.location(m.host, "world") * to_location(m.frame)), _matrix(world), tol=1e-4), key
        joint = F.JOINT_BY_NAME[m.joint]
        z = (world * Location((0.0, 0.0, 1.0))).position - world.position
        assert abs(abs(z.dot(Location((0, 0, 0)).position + __import__("build123d").Vector(*joint.axis_w))) - 1.0) < 1e-6
        assert key in P.DATA["designed_modules"] and key in P.DATA["mounted"]


def test_keys_unique_and_parts_known():
    keys = [o["key"] for o in P.DATA["occurrences"]]
    assert len(keys) == len(set(keys))
    for o in P.DATA["occurrences"]:
        assert o["part"] in (set(R.CUSTOM) | set(R.COTS) | set(R.NATIVE) | set(R.MEASURED) | set(R.MODULES)
                             | set(R.DESIGNED_MODULES)), o["key"]
        assert o["key"] == f"{o['part']}#{o['key'].rsplit('#', 1)[1]}"
        if o["parent"] is not None:
            assert o["parent"] in P.OCCURRENCES and P.OCCURRENCES[o["parent"]]["kind"] == "module"


def test_rotation_convention_round_trips():
    """Location(position, rotation_xyz_deg) must reproduce the stored 3x4 matrix."""
    for o in P.DATA["occurrences"]:
        for frame in ("rel", "world"):
            assert _close(_matrix(P.to_location(o[frame])), o[frame]["matrix_3x4"]), f"{o['key']} {frame}"


def _check_world_equals_parent_world_times_rel():
    for key, o in P.OCCURRENCES.items():
        parent_world = P.location(o["parent"], "world") if o["parent"] else Location()
        # 1e-4 mm/unitless: the JSON stores 6-decimal positions/angles, and a ~1e-6 deg
        # rounding over a ~100 mm lever arm shows up at the 1e-6 level.
        assert _close(_matrix(parent_world * P.location(key, "rel")), _matrix(P.location(key, "world")), tol=1e-4), key


def test_world_equals_parent_world_times_rel():
    _check_world_equals_parent_world_times_rel()


# --- the link-length shifts (lib/placements.py SHIFTS) ---------------------------------------------------------------
def _position_in(anchor: str, key: str) -> tuple:
    """`key`'s world origin in `anchor`'s part frame (both with SHIFTS applied)."""
    return tuple((P.location(anchor, "world").inverse() * P.location(key, "world")).position)


def _near(a, b, tol=1e-3) -> bool:
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b, strict=True))


def _capture_records(*links: str) -> set:
    """The top-level capture records the links' occurrences come from: a gripper part is its module's; the mounted
    records and the designed modules' bodies are not capture records."""
    out = set()
    for link in links:
        for key in RF.LINKS[link]:
            if ":" in key or "mount" in P.OCCURRENCES[key]:
                continue
            out.add(P.OCCURRENCES[key]["parent"] or key)
    return out


def _check_links_carry_what_lies_beyond(elbow_x: float, wrist_x: float, end_dx: float):
    """j2_link#1 (on the elbow axis) sits at (elbow_x, 10, 0) in j1_link's frame, j3_coupler#2 (on the wrist_pitch
    axis) at (wrist_x, 0, 42) in j2_link's; what bolts to the wrist body's end face keeps its capture pose in the
    wrist body's frame but for x + end_dx."""
    assert _near(_position_in("j1_link#1", "j2_link#1"), (elbow_x, 10.0, 0.0))
    assert _near(_position_in("j2_link#1", "j3_coupler#2"), (wrist_x, 0.0, 42.0))
    wrist_raw = P.to_location(P.OCCURRENCES["wrist_link#1"]["world"])
    for key in P.SHIFTS[2].moves:
        raw = tuple((wrist_raw.inverse() * P.to_location(P.OCCURRENCES[key]["world"])).position)
        assert _near(_position_in("wrist_link#1", key), (raw[0] + end_dx, raw[1], raw[2])), key


def test_shift_axes_are_their_anchors_x():
    for s in P.SHIFTS:
        x = [row[0] for row in P.OCCURRENCES[s.anchor]["world"]["matrix_3x4"]]
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(x, s.axis_w, strict=True)), s.link


def test_shifts_move_the_capture_records_beyond_each_link():
    upper_arm, forearm, wrist = (s for s in P.SHIFTS)
    assert (upper_arm.anchor, forearm.anchor, wrist.anchor) == ("j1_link#1", "j2_link#1", "wrist_link#1")
    assert set(upper_arm.moves) == _capture_records("elbow_link", "forearm_link", "wrist_pitch_link", "wrist_roll_link",
                                                    "jaw_a_link", "jaw_b_link") | set(P.RETIRED)
    assert set(forearm.moves) == _capture_records("wrist_pitch_link", "wrist_roll_link", "jaw_a_link", "jaw_b_link") | {
        "gt2_pulley_90t#2"}
    assert set(wrist.moves) == _capture_records("wrist_roll_link", "jaw_a_link", "jaw_b_link") | {
        "gripper_clamp_bracket#1", "nema17_pancake#1"}
    assert all(P.OCCURRENCES[key]["parent"] is None for key in upper_arm.moves)
    assert (upper_arm.along_x, forearm.along_x, wrist.along_x) == (
        UPPER_ARM.DEFAULT.slab.elbow_x - UPPER_ARM.LEGACY.slab.elbow_x,
        FOREARM.DEFAULT.web.wrist_x - FOREARM.LEGACY.web.wrist_x,
        WRIST.DEFAULT.tower.block_x1 - WRIST.LEGACY.tower.block_x1)


def test_the_links_carry_what_lies_beyond_them():
    _check_links_carry_what_lies_beyond(UPPER_ARM.DEFAULT.slab.elbow_x, FOREARM.DEFAULT.web.wrist_x,
                                        WRIST.DEFAULT.tower.block_x1 - WRIST.LEGACY.tower.block_x1)


def test_shifted_links_keep_the_records_consistent(monkeypatch):
    """Shorter links (any lengths): each moves what lies beyond it, a module's children follow it, the rest stays."""
    along = {"j1_link": -40.0, "j2_link": 40.0, "wrist_link": -20.0}
    before = {key: P.location(key, "world") for key in ("base#1", "j1_link#1", "cycloidal_drive#1")}
    monkeypatch.setattr(P, "SHIFTS", tuple(replace(s, along_x=along[s.link]) for s in P.SHIFTS))
    _check_links_carry_what_lies_beyond(UPPER_ARM.LEGACY.slab.elbow_x - 40.0, FOREARM.LEGACY.web.wrist_x + 40.0, -20.0)
    _check_world_equals_parent_world_times_rel()
    for key, loc in before.items():
        assert _close(_matrix(P.location(key, "world")), _matrix(loc), tol=1e-9), key
    for child in (key for key, o in P.OCCURRENCES.items() if o["parent"] == "gripper#1"):
        assert _close(_matrix(P.location(child, "rel")), P.OCCURRENCES[child]["rel"]["matrix_3x4"], tol=1e-6), child


def test_assembly_tables_claim_every_part_key_exactly_once():
    used = [key for _, _, key in arm.OCCURRENCES if key not in arm.MODULE_KEYS] + [key for _, _, key in gripper.OCCURRENCES]
    assert sorted(used) == sorted(P.keys(kind="part"))
    assert not set(P.RETIRED) & set(used), "a retired occurrence is back in a table"
    assert [key for _, _, key in arm.OCCURRENCES if key in arm.MODULE_KEYS] == ["cycloidal_drive#1", "forearm_roll_drive#1", "gripper#1"]
    assert all(P.OCCURRENCES[key]["parent"] == "gripper#1" for _, _, key in gripper.OCCURRENCES)
    assert all(P.OCCURRENCES[key]["parent"] is None for _, _, key in arm.OCCURRENCES)
    for name, _, key in arm.OCCURRENCES + gripper.OCCURRENCES:
        assert P.OCCURRENCES[key]["part"] == name, f"{key} is not a {name} occurrence"


def test_duplicate_parts_have_unique_labels():
    for rows in (arm.OCCURRENCES, gripper.OCCURRENCES, cycloidal_drive.OCCURRENCES, forearm_roll_drive.OCCURRENCES):
        labels = [name if role is None else f"{name}:{role}" for name, role, _ in rows]
        assert len(labels) == len(set(labels)), f"duplicate labels: {labels}"


def test_designed_module_records_the_cycloidal_drive():
    """The drive's SolidWorks node is a designed module: pose from SolidWorks (verbatim the
    former skipped[0]), contents from assemblies/cycloidal_drive.py."""
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
    assert "cycloidal_drive" not in {s["label"] for s in P.DATA["skipped"]}


def test_skipped_nodes_are_the_dropped_products():
    """A SolidWorks node the design dropped (lib/reference.py SKIPPED_PRODUCTS - the link caps since 2026-09-25) is
    no occurrence: its pose stays under `skipped` (tools/reference/mount_placements.py's merge mode moved them there,
    as an extraction would write them)."""
    skipped = P.DATA["skipped"]
    assert [s["label"] for s in skipped] == ["cap_1_joint_2_8726", "cap_of_joint_2_piece_2_8526", "first_joint_cap_8726"]
    assert [s["path"] for s in skipped] == ["1.15", "1.16", "1.17"]
    assert all(s["reason"] == R.SKIPPED_LABELS[s["label"]] and (s["leaves"], s["solids"]) == (1, 1) for s in skipped)
    assert not {o["label_in_monolith"] for o in P.DATA["occurrences"]} & set(R.SKIPPED_LABELS)
    for s in skipped:
        assert _close(_matrix(P.to_location(s["rel"])), s["rel"]["matrix_3x4"]) and s["rel"] == s["world"]   # top-level nodes
