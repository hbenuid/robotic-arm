"""tools/bom.py - the print list and the buy list come from the assembly tables and the one make/buy
label (parts.bought(): COTS = True). Fast: metadata only, no geometry, no CAD kernel."""
import parts
from assemblies import arm, cycloidal_drive, gripper
from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG as CFG
from tools import bom


def test_every_part_is_counted_once_per_occurrence():
    counts = bom.part_counts()
    leaves = len(arm.OCCURRENCES) - len(arm.MODULES) + len(gripper.OCCURRENCES) + len(cycloidal_drive.OCCURRENCES)
    assert sum(counts.values()) == leaves == 59
    assert set(counts) == set(parts.names()), "a part under parts/ that no assembly places (or the reverse)"
    assert sum(bom.part_counts("gripper").values()) == len(gripper.OCCURRENCES)
    assert sum(bom.part_counts("cycloidal_drive").values()) == len(cycloidal_drive.OCCURRENCES)


def test_the_two_lists_partition_the_parts_by_the_cots_flag():
    printed, bought = bom.print_rows(), bom.buy_rows()
    assert {r["part"] for r in printed} | {r["part"] for r in bought} == set(parts.names())
    assert not {r["part"] for r in printed} & {r["part"] for r in bought}
    assert {r["part"] for r in bought} == set(R.COTS)
    assert (len(printed), sum(r["qty"] for r in printed)) == (26, 34)
    assert (len(bought), sum(bom.part_counts()[r["part"]] for r in bought)) == (17, 25)
    assert {r["state"] for r in printed} <= {"wrapper", "parametric", "designed"}
    assert {r["part"] for r in printed if r["state"] == "designed"} == set(R.DESIGNED)


def test_drive_buy_list_matches_the_drive_config():
    pieces = {r["part"]: r["pieces"] for r in bom.buy_rows("cycloidal_drive")}
    assert set(pieces) == set(R.CYCLOIDAL_COTS) | {"mks_servo42d"}   # the kit's board rides the drive motor (parts/joints/)
    assert pieces == {
        "bearing_6003": 2, "bearing_6814": CFG.bearings.out_qty, "bearing_625": 1, "nema17_48mm": 1, "mks_servo42d": 1,
        "cycloidal_ring_pins": CFG.gear.num_ring_pins, "cycloidal_output_pins": CFG.disc.output_pin_count,
        "cycloidal_shaft_support_pin": 1, "cycloidal_motor_bolts": 4,
        "cycloidal_housing_bolts": CFG.housing.bolt_count, "cycloidal_housing_nuts": CFG.housing.bolt_count,
    }
    assert {r["part"] for r in bom.print_rows("cycloidal_drive")} == set(R.DESIGNED)
    assert [r["part"] for r in bom.buy_rows("cycloidal_drive") if r["geometry"] == "vendor"] == ["bearing_625", "mks_servo42d", "nema17_48mm"]


def test_extras_are_well_formed_and_scoped_to_a_module():
    assert bom.EXTRAS, "the arm-mount fasteners and the grease are purchased but not modelled - keep them listed"
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
    assert "## PRINT - 6 parts" in out and "## BUY - 11 parts, 53 pieces" in out and "## BUY, NOT MODELLED" in out
