"""The arm assembly rebuilt from parts + placements reproduces the SolidWorks totals (the designed modules counted by
their own locks, which their own tests hold: tests/cycloidal/test_assembly.py, tests/forearm/test_roll_drive.py) and the
colors: in every assembly a purchased part is BOUGHT_TINT grey, a printed one its link's / module's tint (the
standalone drives' in their own tests)."""
import importlib
import pathlib

import pytest
from build123d import Location, Vector

import parts
from assemblies import arm, cycloidal_drive, forearm_roll_drive, gripper
from assemblies._occurrences import BOUGHT_TINT
from lib import placements as P
from lib import reference as R
from lib.models import raw
from tests import built
from tests.helpers import leaves, module_tints, same_color
from tests.source_checks import runs_its_model
from tests.totals import part_totals, world_bbox


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
            for leaf in leaves(member):
                bought = parts.bought(leaf.label.split(":")[0])
                assert same_color(leaf, BOUGHT_TINT if bought else printed), f"{group.label}/{leaf.label}: {tuple(leaf.color)}"
                seen[bought] += 1
    return seen


def _expected_totals():
    """(leaves, solids, volume) of the arm: every part occurrence of placements.json (tests.totals: the
    SolidWorks record, or the part's own build once converted), plus the designed modules' own totals (their
    EXPECTED)."""
    keys = P.keys(kind="part")
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
        bb = built.placed(k).bounding_box()
        boxes.append(((bb.min.X, bb.min.Y, bb.min.Z), (bb.size.X, bb.size.Y, bb.size.Z)))
    for bmin, bsize in boxes:
        for i in range(3):
            lo[i] = min(lo[i], bmin[i])
            hi[i] = max(hi[i], bmin[i] + bsize[i])
    corners = [tuple((arm.arm_from_w() * Location(tuple(p))).position) for p in (lo, hi)]
    lo = [min(c[i] for c in corners) for i in range(3)]
    hi = [max(c[i] for c in corners) for i in range(3)]
    return lo, [h - l for h, l in zip(hi, lo, strict=True)]


@pytest.mark.slow
def test_gripper_module_builds():
    g = raw(gripper.gripper)
    assert g.label == "gripper"
    assert len(leaves(g)) == len(gripper.OCCURRENCES) == 19
    assert module_tints(g, gripper.TINT) == {True: 4, False: 15}   # servo, horn, 2 rails


def test_every_assembly_runs_its_model():
    """Each assemblies/<name>.py ends with the `__main__` call of its model <name>() - without it
    `./cadtool gen assemblies/<name>.py` builds nothing and the module is only ever built as the arm's child."""
    files = sorted(p for p in pathlib.Path(arm.__file__).parent.glob("*.py") if not p.stem.startswith("_"))
    assert files
    for path in files:
        assert runs_its_model(path, path.stem), f"{path.name}: must end with `if __name__ == \"__main__\": {path.stem}()`"


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
    arm_leaves = leaves(a)
    exp_leaves, exp_solids, exp_volume = _expected_totals()
    assert len(arm_leaves) == exp_leaves == 85
    labels = [leaf.label for leaf in arm_leaves]
    assert len(set(labels)) == len(labels), f"duplicate leaf labels: {labels}"
    assert len(a.solids()) == exp_solids == 244
    assert abs(R.solid_volume(a) - exp_volume) <= 0.5
    exp_min, exp_size = _expected_bbox()
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_min(a), exp_min, strict=True)), (R.bbox_min(a), exp_min)
    assert all(abs(x - y) <= 0.05 for x, y in zip(R.bbox_size(a), exp_size, strict=True)), (R.bbox_size(a), exp_size)
    assert _check_link_tints(a) == {True: 49, False: 36}   # bought / printed leaves (tools/bom.py counts the same)
