"""tools/bom.py - the print list and the buy list come from the assembly tables and the one make/buy
label (parts.bought(): COTS = True). Fast: metadata only, no geometry, no CAD kernel."""
import parts
from assemblies import arm, cycloidal_drive, forearm_roll_drive, gripper
from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG as CFG
from tools import bom


def test_every_part_is_counted_once_per_occurrence():
    counts = bom.part_counts()
    leaves = (len(arm.OCCURRENCES) - len(arm.MODULES) + len(gripper.OCCURRENCES) + len(cycloidal_drive.OCCURRENCES)
              + len(forearm_roll_drive.OCCURRENCES))
    assert sum(counts.values()) == leaves == 87
    assert set(counts) == set(parts.names()) - set(parts.unplaced()), "a part under parts/ that no assembly places (or the reverse)"
    assert not set(counts) & set(parts.unplaced()), "a placed part still declares UNPLACED - drop it (and its EXTRAS row)"
    assert sum(bom.part_counts("gripper").values()) == len(gripper.OCCURRENCES)
    assert sum(bom.part_counts("cycloidal_drive").values()) == len(cycloidal_drive.OCCURRENCES)
    assert sum(bom.part_counts("forearm_roll_drive").values()) == len(forearm_roll_drive.OCCURRENCES)


def test_the_two_lists_partition_the_parts_by_the_cots_flag():
    printed, bought = bom.print_rows(), bom.buy_rows()
    assert {r["part"] for r in printed} | {r["part"] for r in bought} == set(parts.names()) - set(parts.unplaced())
    assert not {r["part"] for r in printed} & {r["part"] for r in bought}
    assert {r["part"] for r in bought} == (set(R.COTS) | {n for n in R.NO_REFERENCE if parts.bought(n)}) - set(parts.unplaced())
    assert (len(printed), sum(r["qty"] for r in printed)) == (31, 38)
    assert (len(bought), sum(bom.part_counts()[r["part"]] for r in bought)) == (31, 49)
    assert {r["state"] for r in printed} <= {"wrapper", "parametric", "designed", "native", "measured", "no reference"}
    assert {r["part"] for r in printed if r["state"] == "no reference"} == {n for n in R.NO_REFERENCE if not parts.bought(n)}
    assert {r["part"] for r in printed if r["state"] == "designed"} == set(R.DESIGNED)
    assert {r["part"] for r in printed if r["state"] == "native"} == set(R.NATIVE)


def test_drive_buy_list_matches_the_drive_config():
    pieces = {r["part"]: r["pieces"] for r in bom.buy_rows("cycloidal_drive")}
    assert set(pieces) == set(R.CYCLOIDAL_COTS) | {"mks_servo42d"}   # the kit's board rides the drive motor (parts/joints/)
    assert pieces == {
        "bearing_6003": 2, "bearing_6814": CFG.bearings.out_qty, "bearing_625": 1, "nema17_48mm": 1, "mks_servo42d": 1,
        "cycloidal_ring_pins": CFG.gear.num_ring_pins, "cycloidal_output_pins": CFG.disc.output_pin_count,
        "cycloidal_shaft_support_pin": 1, "cycloidal_motor_bolts": 4,
        "cycloidal_housing_bolts": CFG.housing.bolt_count, "cycloidal_housing_nuts": CFG.housing.bolt_count,
    }
    assert {r["part"] for r in bom.print_rows("cycloidal_drive")} == set(R.DESIGNED) | {"cycloidal_shell_ring"}
    assert [r["part"] for r in bom.buy_rows("cycloidal_drive") if r["geometry"] == "vendor"] == ["bearing_625", "mks_servo42d", "nema17_48mm"]


def test_roll_drive_lists_follow_its_rows():
    pieces = {r["part"]: r["pieces"] for r in bom.buy_rows("forearm_roll_drive")}
    assert pieces == {"bearing_6808": 2, "nema17_40mm": 1, "mks_servo42d": 1, "gt2_pulley_20t": 1,
                      "forearm_roll_mount_screws": 4, "forearm_roll_mount_nuts": 4}
    assert {r["part"] for r in bom.print_rows("forearm_roll_drive")} == set(R.NATIVE) - {"base_motor_mount"} == {
        "forearm_roll_block", "forearm_roll_shaft", "forearm_roll_retainer", "forearm_roll_motor_mount"}
    assert all(r["state"] == "native" for r in bom.print_rows("forearm_roll_drive"))
    assert [r["geometry"] for r in bom.buy_rows("forearm_roll_drive") if r["part"] == "bearing_6808"] == ["envelope"]


def test_the_belt_joints_take_a_6806_pair_each():
    """base_yaw, elbow_pitch, wrist_pitch: two 6806-2RS each (lib/mounts.py BEARING_MOUNTS), no vendor model."""
    row = next(r for r in bom.buy_rows() if r["part"] == "bearing_6806")
    assert (row["pieces"], row["geometry"]) == (6, "envelope")
    assert row["order"].startswith("6806-2RS (61806)") and row["order"].endswith("30 x 42 x 7")


def test_the_90t_pulleys_take_their_m4_screws_and_nuts():
    """Each belt joint's driven pulley (the elbow's and the wrist's 90T, the base_yaw 120T) is clamped by 4x M4 + nuts
    (lib/mounts.py FASTENER_MOUNTS): modelled pattern parts on the
    buy list, no vendor model - no longer EXTRAS rows."""
    rows = {r["part"]: r for r in bom.buy_rows() if r["part"].endswith(("_pulley_screws", "_pulley_nuts"))}
    assert set(rows) == {"elbow_pulley_screws", "elbow_pulley_nuts", "wrist_pulley_screws", "wrist_pulley_nuts",
                         "yaw_pulley_screws", "yaw_pulley_nuts"}
    assert all((r["pieces"], r["geometry"]) == (4, "envelope") for r in rows.values())
    assert rows["elbow_pulley_screws"]["order"].startswith("M4 x 40 socket head cap screw")
    assert rows["wrist_pulley_screws"]["order"].startswith("M4 x 50 socket head cap screw")
    assert rows["yaw_pulley_screws"]["order"].startswith("M4 x 45 socket head cap screw")
    assert all(rows[n]["order"] == "M4 hex nut (ISO 4032)" for n in ("elbow_pulley_nuts", "wrist_pulley_nuts", "yaw_pulley_nuts"))
    driven = {r["part"]: r["qty"] for r in bom.print_rows() if r["part"] in ("gt2_pulley_90t", "gt2_pulley_120t")}
    assert driven == {"gt2_pulley_90t": 2, "gt2_pulley_120t": 1} and sum(driven.values()) == len(rows) // 2   # a pulley per bolted joint
    assert not any("M4" in spec for owner, spec, _, _ in bom.EXTRAS if owner == "forearm_roll_drive")


def test_the_roll_motor_mount_takes_its_m3_screws_and_nuts():
    """The roll motor mount is clamped into the elbow block's step by 4x M3 countersunk + nuts (the module's rows):
    modelled pattern parts on the buy list, no vendor model - no EXTRAS row."""
    rows = {r["part"]: r for r in bom.buy_rows("forearm_roll_drive") if r["part"].startswith("forearm_roll_mount_")}
    assert set(rows) == {"forearm_roll_mount_screws", "forearm_roll_mount_nuts"}
    assert all((r["pieces"], r["geometry"]) == (4, "envelope") for r in rows.values())
    assert rows["forearm_roll_mount_screws"]["order"].startswith("M3 x 16 countersunk socket screw (ISO 10642)")
    assert rows["forearm_roll_mount_nuts"]["order"].startswith("M3 hex nut (ISO 4032)")
    assert not any(("countersunk" in spec or "M3 hex nut" in spec) and "mount" in spec
                   for owner, spec, _, _ in bom.EXTRAS if owner == "forearm_roll_drive")


def test_the_base_motor_mount_takes_its_m4_screws_and_nuts():
    """The base's bolt-on motor mount is held to the base by 4x M4 SHCS + nuts (lib/mounts.py BASE_MOUNTS): the mount
    on the print list (native), the screws and nuts on the buy list, no vendor model - no EXTRAS row."""
    assert [r["state"] for r in bom.print_rows() if r["part"] == "base_motor_mount"] == ["native"]
    rows = {r["part"]: r for r in bom.buy_rows() if r["part"].startswith("base_motor_mount_")}
    assert set(rows) == {"base_motor_mount_screws", "base_motor_mount_nuts"}
    assert all((r["pieces"], r["geometry"]) == (4, "envelope") for r in rows.values())
    assert rows["base_motor_mount_screws"]["order"].startswith("M4 x 20 socket head cap screw (ISO 4762)")
    assert rows["base_motor_mount_nuts"]["order"].startswith("M4 hex nut (ISO 4032)")


def test_extras_are_well_formed_and_scoped_to_a_module():
    assert bom.EXTRAS, "the hub's and the clamp cap's fasteners and the grease are purchased but not modelled - keep them listed"
    for owner, spec, pieces, why in bom.EXTRAS:
        assert owner is None or owner in arm.MODULES
        assert isinstance(spec, str) and spec.strip() and isinstance(why, str) and why.strip()
        assert isinstance(pieces, int) and pieces >= 1
    assert len(bom.extra_rows()) == len(bom.EXTRAS)
    assert all(r["module"] == "cycloidal_drive" for r in bom.extra_rows("cycloidal_drive"))
    assert bom.extra_rows("gripper") == []


def test_cli_prints_the_three_lists(capsys):
    assert bom.main(["--module", "cycloidal_drive", "--md"]) == 0
    out = capsys.readouterr().out
    assert "## PRINT - 7 parts" in out and "## BUY - 11 parts, 49 pieces" in out and "## BUY, NOT MODELLED" in out
