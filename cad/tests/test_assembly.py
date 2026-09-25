"""The arm assembly rebuilt from parts + placements reproduces the SolidWorks totals, plus the
code-driven cycloidal_drive module's own totals, the caps-off working view (arm_no_caps) and the colors:
in every assembly a purchased part is BOUGHT_TINT grey, a printed one its link's / module's tint."""
import importlib
import pathlib

import pytest
from build123d import Color, Location, Vector

import parts
from assemblies import arm, arm_no_caps, cycloidal_drive, forearm_roll_drive, gripper
from assemblies._occurrences import BOUGHT_TINT
from lib import placements as P
from lib import reference as R
from lib.models import raw
from tests.source_checks import runs_its_model
from tests.totals import part_totals, world_bbox

DRIVE = cycloidal_drive.EXPECTED
ROLL = forearm_roll_drive.EXPECTED


def _leaves(node):
    return [node] if not node.children else [leaf for child in node.children for leaf in _leaves(child)]


def _same_color(shape, tint: str) -> bool:
    return all(abs(x - y) < 1e-6 for x, y in zip(tuple(shape.color), tuple(Color(tint))))


def _check_module_tints(module, tint):
    """A standalone module: purchased parts BOUGHT_TINT, printed parts the module's TINT; returns the
    bought / printed leaf counts."""
    seen = {True: 0, False: 0}
    for leaf in _leaves(module):
        bought = parts.bought(leaf.label.split(":")[0])
        assert _same_color(leaf, BOUGHT_TINT if bought else tint), f"{module.label}/{leaf.label}: {tuple(leaf.color)}"
        seen[bought] += 1
    return seen


def _check_link_tints(root):
    """The arm's colors: a purchased part (parts.bought()) is BOUGHT_TINT wherever it sits - inside the
    gripper and the drive too -, a printed part carries its module's MODULE_TINTS color or else its
    link group's."""
    module_tint = {key.split("#")[0]: tint for key, tint in arm.MODULE_TINTS.items()}
    group_tint = {label: tint for label, tint, _ in arm.GROUPS}
    seen = {True: 0, False: 0}
    for group in root.children:
        for member in group.children:
            printed = module_tint.get(member.label, group_tint[group.label])
            for leaf in _leaves(member):
                bought = parts.bought(leaf.label.split(":")[0])
                assert _same_color(leaf, BOUGHT_TINT if bought else printed), f"{group.label}/{leaf.label}: {tuple(leaf.color)}"
                seen[bought] += 1
    return seen


def _expected_totals(hidden=()):
    """(leaves, solids, volume) of the arm: every part occurrence of placements.json (tests.totals: the
    SolidWorks record, or the part's own build once converted) but the `hidden` keys, plus the designed
    modules' own totals (their EXPECTED)."""
    keys = [k for k in P.keys(kind="part") if k not in set(hidden)]
    solids, volume = part_totals(keys)
    leaves = len(keys)
    for k in P.keys(kind="module", designed=True):
        lock = importlib.import_module(f"assemblies.{P.OCCURRENCES[k]['part']}").EXPECTED
        leaves, solids, volume = leaves + lock["leaves"], solids + lock["solids"], volume + lock["solid_volume"]
    return leaves, solids, volume


def _expected_bbox():
    """Union of the world bounding boxes of every part occurrence in placements.json (the mounted
    motors + boards included; a converted part's from its build - tests.totals) and of the designed
    modules built in-process at their world pose (the SolidWorks node's box predates the drive's MKS
    board), re-expressed in the frame the arm is emitted in (arm.arm_from_w(): W, +Y up -> base_link,
    Z up). That map is an axis permutation with signs, so two opposite corners carry the whole box."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    boxes = [world_bbox(k) for k in P.keys(kind="part")]
    for k in P.keys(kind="module", designed=True):
        bb = raw(arm.MODULES[P.OCCURRENCES[k]["part"]]).moved(P.location(k, "world")).bounding_box()
        boxes.append(((bb.min.X, bb.min.Y, bb.min.Z), (bb.size.X, bb.size.Y, bb.size.Z)))
    for bmin, bsize in boxes:
        for i in range(3):
            lo[i] = min(lo[i], bmin[i])
            hi[i] = max(hi[i], bmin[i] + bsize[i])
    corners = [tuple((arm.arm_from_w() * Location(tuple(p))).position) for p in (lo, hi)]
    lo = [min(c[i] for c in corners) for i in range(3)]
    hi = [max(c[i] for c in corners) for i in range(3)]
    return lo, [h - l for h, l in zip(hi, lo)]


@pytest.mark.slow
def test_gripper_module_builds():
    g = raw(gripper.gripper)
    assert g.label == "gripper"
    assert len(_leaves(g)) == len(gripper.OCCURRENCES) == 19
    assert g.is_valid
    assert _check_module_tints(g, gripper.TINT) == {True: 4, False: 15}   # servo, horn, 2 rails


@pytest.mark.slow
def test_cycloidal_drive_module_builds():
    d = raw(cycloidal_drive.cycloidal_drive)
    assert d.label == "cycloidal_drive"
    assert len(_leaves(d)) == len(cycloidal_drive.OCCURRENCES) == DRIVE["leaves"] == 19
    assert len(d.solids()) == DRIVE["solids"]
    assert abs(R.solid_volume(d) - DRIVE["solid_volume"]) <= 0.5
    assert d.is_valid
    assert _check_module_tints(d, cycloidal_drive.TINT) == {True: 13, False: 6}   # + the MKS board


@pytest.mark.slow
def test_forearm_roll_drive_module_builds():
    d = raw(forearm_roll_drive.forearm_roll_drive)
    assert d.label == "forearm_roll_drive"
    assert len(_leaves(d)) == len(forearm_roll_drive.OCCURRENCES) == ROLL["leaves"] == 8
    assert len(d.solids()) == ROLL["solids"]
    assert abs(R.solid_volume(d) - ROLL["solid_volume"]) <= 0.5
    assert d.is_valid
    assert _check_module_tints(d, forearm_roll_drive.TINT) == {True: 5, False: 3}   # 2 bearings, motor, board, 20T / block, shaft, retainer


def test_arm_groups_mirror_links():
    """GROUPS covers every occurrence key exactly once and mirrors robot/frames.py LINKS with the
    three modules kept whole: wrist = wrist_roll_link + jaw_a_link + jaw_b_link, the cycloidal
    drive sits under shoulder_link although LINKS puts its rotor body in upper_arm_link, the forearm
    roll drive under elbow_link although LINKS puts its rotor (the shaft) in forearm_link."""
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
    assert groups["elbow_link"] == whole("elbow_link")
    assert groups["forearm_link"] == whole("forearm_link") - {arm.ROLL_KEY}
    assert groups["wrist_pitch_link"] == set(frames.LINKS["wrist_pitch_link"])
    wrist = (groups["wrist"] - {arm.GRIPPER_KEY}) | {key for _, _, key in gripper.OCCURRENCES}
    assert wrist == set(
        frames.LINKS["wrist_roll_link"] + frames.LINKS["jaw_a_link"] + frames.LINKS["jaw_b_link"]
    )


def test_grey_means_bought():
    """BOUGHT_TINT is reserved for the purchased parts: no link group or module may be tinted with it
    (base_link used to be grey); the two modules are the same color standalone and in the arm."""
    tints = [tint for _, tint, _ in arm.GROUPS] + list(arm.MODULE_TINTS.values())
    assert BOUGHT_TINT.lower() not in {t.lower() for t in tints}
    assert len({t.lower() for t in tints}) == len(tints), "two groups / modules share a tint"
    assert arm.MODULE_TINTS == {arm.DRIVE_KEY: cycloidal_drive.TINT, arm.ROLL_KEY: forearm_roll_drive.TINT, arm.GRIPPER_KEY: gripper.TINT}


def test_arm_is_emitted_z_up():
    """The arm leaves the SolidWorks capture frame W (+Y up) for the base_link frame of
    robot/frames.py (REP-103) - cadgen's viewer and snapshots are Z-up, a W-frame arm.step lies on
    its side: world up -> +Z, the arm's forward -> +X, the base's mounting face centre -> origin."""
    from robot import frames

    def direction(v):
        return (arm.arm_from_w() * Location(v)).position - arm.arm_from_w().position

    assert (direction(frames.U) - Vector(0, 0, 1)).length < 1e-9
    assert (direction(frames.BASE_FORWARD) - Vector(1, 0, 0)).length < 1e-9
    assert (arm.arm_from_w() * Location((0.0, frames.BASE_BOTTOM_Y, 0.0))).position.length < 1e-9


@pytest.mark.slow
def test_arm_assembly_matches_reference_totals():
    """SolidWorks totals for the SolidWorks-driven occurrences + the drive module's own totals,
    under the GROUPS component tree (arm -> base_link/shoulder_link/upper_arm_link/forearm_link/
    wrist_pitch_link/wrist)."""
    a = raw(arm.arm)
    assert a.label == "arm"
    assert [c.label for c in a.children] == [label for label, _, _ in arm.GROUPS]
    leaves = _leaves(a)
    exp_leaves, exp_solids, exp_volume = _expected_totals()
    assert len(leaves) == exp_leaves == 66
    labels = [leaf.label for leaf in leaves]
    assert len(set(labels)) == len(labels), f"duplicate leaf labels: {labels}"
    assert len(a.solids()) == exp_solids == 199
    assert abs(R.solid_volume(a) - exp_volume) <= 0.5
    exp_min, exp_size = _expected_bbox()
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_min(a), exp_min)), (R.bbox_min(a), exp_min)
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_size(a), exp_size)), (R.bbox_size(a), exp_size)
    assert a.is_valid
    assert _check_link_tints(a) == {True: 30, False: 36}   # bought / printed leaves (tools/bom.py counts the same)


def test_arm_no_caps_tables_are_the_arms_minus_hidden():
    """The caps-off working view derives its tables from arm.py: every HIDDEN key is an occurrence of
    the arm (a renamed key must not silently bring a cap back), everything else is kept in order, and
    the file ends with its build call (without it `./cadtool gen` builds nothing)."""
    hidden = set(arm_no_caps.HIDDEN)
    assert len(hidden) == len(arm_no_caps.HIDDEN) == 3
    assert hidden <= {key for _, _, key in arm.OCCURRENCES}
    assert all(P.OCCURRENCES[key]["part"].endswith(("_cap", "_cap_1", "_cap_2")) for key in hidden)
    assert arm_no_caps.OCCURRENCES == [row for row in arm.OCCURRENCES if row[2] not in hidden]
    assert [(label, tint) for label, tint, _ in arm_no_caps.GROUPS] == [(label, tint) for label, tint, _ in arm.GROUPS]
    for (label, _, keys), (_, _, full_keys) in zip(arm_no_caps.GROUPS, arm.GROUPS):
        assert keys == tuple(key for key in full_keys if key not in hidden), label
        assert keys, f"{label}: hiding must not empty a group"
    assert runs_its_model(pathlib.Path(arm_no_caps.__file__), "arm_no_caps")


@pytest.mark.slow
def test_arm_no_caps_builds_the_arm_without_its_caps():
    a = raw(arm_no_caps.arm_no_caps)
    assert a.label == "arm_no_caps"
    assert [c.label for c in a.children] == [label for label, _, _ in arm.GROUPS]
    labels = [leaf.label for leaf in _leaves(a)]
    exp_leaves, exp_solids, exp_volume = _expected_totals(hidden=arm_no_caps.HIDDEN)
    assert len(labels) == exp_leaves == 63
    hidden = [P.OCCURRENCES[key] for key in arm_no_caps.HIDDEN]
    assert not {o["part"] for o in hidden} & set(labels), labels
    assert len(a.solids()) == exp_solids == 196
    assert abs(R.solid_volume(a) - exp_volume) <= 0.5
    assert a.is_valid
    assert _check_link_tints(a) == {True: 30, False: 36 - len(arm_no_caps.HIDDEN)}   # the caps are printed
