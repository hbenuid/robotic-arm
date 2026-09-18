"""The `@step` model convention, enforced automatically for every parts/<group>/<name>.py.

Auto-discovers parts exactly like the assemblies do (parts.names(): every module under a group
package that does not start with '_') and checks each honours the cadgen 0.5 contract: importable
with no side effects, ONE `@step def <name>()` model named after the file whose STEP is the
sibling file (no out=), whose body (parts.build(name) - never the model itself, that would start a
build) returns a valid, non-empty, labelled shape, plus the wrapper / designed / COTS metadata this
repo adds. New parts are covered the moment they land.
"""
import pytest

import parts
from lib import reference as R
from tests.source_checks import runs_its_model

PART_NAMES = parts.names()

# Parts that are DELIBERATELY several disjoint solids (multi-body SolidWorks parts / vendor
# files): name -> expected solid count. Everything else must be exactly one solid.
MULTI_BODY = {
    "gripper_j3_connector": 2,
    "gt2_pulley_20t": 3,
    "mg996r_servo": 4,
    "nema17_pancake": 11,
    # cycloidal drive fastener / pin patterns (one compound per pattern)
    "cycloidal_ring_pins": 21,
    "cycloidal_output_pins": 4,
    "cycloidal_motor_bolts": 4,
    "cycloidal_housing_bolts": 8,
    "cycloidal_housing_nuts": 8,
}


def test_some_parts_were_discovered():
    assert PART_NAMES, "no parts discovered under parts/"


def test_naming_map_matches_part_files():
    expected = set(R.CUSTOM) | set(R.COTS) | set(R.DESIGNED)
    assert set(PART_NAMES) == expected, (
        f"parts/ and lib/reference.py disagree: only in parts/: {set(PART_NAMES) - expected}; "
        f"only in the map: {expected - set(PART_NAMES)}"
    )


def test_cycloidal_group_is_the_drive():
    """parts/cycloidal/ holds exactly the drive's parts (their references live in reference/cycloidal/)."""
    assert {n for n, g in parts.GROUPS.items() if g == "cycloidal"} == R.CYCLOIDAL_PARTS
    assert set(parts.GROUPS.values()) == {"base", "joints", "wrist", "gripper", "cycloidal"}


@pytest.mark.parametrize("name", PART_NAMES)
def test_part_declares_its_contract(name):
    """Fast: metadata only, no geometry."""
    mod = parts.load(name)
    assert not hasattr(mod, "gen_step"), f"parts.{name}: gen_step() is the pre-0.5 convention - decorate `def {name}()` with @step"
    m = parts.model(name)   # raises unless `@step def <name>()` exists
    assert m.__name__ == name
    assert m.__cadgen_model__.fmt == "step", f"parts.{name}: the model must be a @step model"
    assert m.__cadgen_model__.out is None, f"parts.{name}: no out= - the STEP is the sibling parts/<group>/{name}.step"
    assert runs_its_model(parts.source_of(name), name), (
        f"parts.{name}: the file must end with `if __name__ == \"__main__\": {name}()` - without it `./cadtool gen` builds nothing")
    if getattr(mod, "COTS", False):
        assert name in R.COTS, f"{name} declares COTS but is not in lib.reference.COTS"
        mass = getattr(mod, "MASS_G", None)
        assert isinstance(mass, (int, float)) and mass > 0, f"COTS part {name} must set MASS_G > 0"
        assert mod.VENDOR_STEP == R.VENDOR_DIR / f"{name}.step", f"{name}.VENDOR_STEP must be vendor/{name}.step"
    else:
        assert name in R.CUSTOM or name in R.DESIGNED, f"{name} is not in lib.reference.CUSTOM / DESIGNED (and not COTS)"
        assert getattr(mod, "REFERENCE", None) == name, f"{name}.REFERENCE must name reference/<origin>/{name}.step"
        assert isinstance(getattr(mod, "CONVERTED", None), bool), f"{name} must declare CONVERTED = True/False"
        assert R.path_of(mod.REFERENCE).exists(), f"missing {R.path_of(name)} (run tools/reference/import_reference.py or tools/cycloidal/import_reference.py)"


@pytest.mark.slow
@pytest.mark.parametrize("name", PART_NAMES)
def test_part_builds_valid_labelled_geometry(name):
    shape = parts.build(name)
    assert shape is not None, f"{name}() returned None"
    assert shape.label == name, f"{name} must label its result with its own name, got {shape.label!r}"
    assert shape.is_valid, f"{name} built an invalid shape"
    assert R.solid_volume(shape) > 0, f"{name} built an empty/zero-volume shape"
    got, expected = len(shape.solids()), MULTI_BODY.get(name, 1)
    assert got == expected, (
        f"{name} built {got} disjoint solids, expected {expected}. A stray solid usually means a "
        f"cutter helper leaked geometry into the active BuildPart; if this part is intentionally "
        f"multi-body, register it in MULTI_BODY."
    )


@pytest.mark.slow
@pytest.mark.parametrize("name", PART_NAMES)
def test_part_survives_cadgen_component_round_trip(name):
    """cadgen stores every component as BinTools BREP bytes (format VERSION_4) and reads them back
    to write the STEP and to link children. OCCT 7.8.1's reader mis-framed those streams for 11 of
    our parts (the reason this project is on OCP 7.9) - this locks that the writer in use round-trips
    every part."""
    from cadgen._internal import component_package as cp

    shape = parts.build(name)
    back = cp._build123d_shape_from_brep_bytes(cp._shape_brep_bytes(shape))
    assert back.is_valid and len(back.solids()) == len(shape.solids())
    assert abs(R.solid_volume(back) - R.solid_volume(shape)) < 1e-6 * max(1.0, R.solid_volume(shape))


COTS_PARTS = [n for n in PART_NAMES if n in R.COTS]


@pytest.mark.parametrize("name", COTS_PARTS)
def test_cots_envelope_tracks_reference_bbox(name):
    """The fallback envelope must occupy the vendor geometry's bounding box (same frame)."""
    import json

    mod = parts.load(name)
    entry = json.loads((R.REF_DIR / "manifest.json").read_text(encoding="utf-8"))["parts"][name]
    env = mod._envelope()
    assert env.is_valid
    for got, exp in ((R.bbox_min(env), entry["bbox_min"]), (R.bbox_size(env), entry["bbox_size"])):
        assert all(abs(g - e) <= 0.05 for g, e in zip(got, exp)), f"{name} envelope {got} vs reference {exp}"


COTS_FRAME_TOL_MM = 1.5   # catalog models differ slightly from the SolidWorks re-exports


@pytest.mark.slow
@pytest.mark.parametrize("name", COTS_PARTS)
def test_cots_vendor_matches_reference_frame(name):
    """The vendor geometry (after VENDOR_TO_REF) must occupy the SolidWorks reference's bounding
    box - guards the re-orientation of a swapped-in step.parts model."""
    mod = parts.load(name)
    if not mod.VENDOR_STEP.exists():
        pytest.skip(f"no vendor/{name}.step - envelope in use")
    shape = parts.build(name)
    ref = R.load(name)
    for got, exp, what in (
        (R.bbox_min(shape), R.bbox_min(ref), "bbox min"),
        (R.bbox_size(shape), R.bbox_size(ref), "bbox size"),
    ):
        assert all(abs(g - e) <= COTS_FRAME_TOL_MM for g, e in zip(got, exp)), (
            f"{name} vendor geometry {what} {got} vs reference {exp} (tol {COTS_FRAME_TOL_MM} mm) - "
            f"set VENDOR_TO_REF in parts/{name}.py"
        )
