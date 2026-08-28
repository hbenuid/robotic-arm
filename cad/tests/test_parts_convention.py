"""The gen_step() convention, enforced automatically for every parts/*.py.

Auto-discovers parts exactly like the assemblies do (every parts/*.py not starting with
'_') and checks each honours the plugin-native contract: importable with no side effects,
a callable gen_step() returning a valid, non-empty, labelled shape, plus the wrapper /
designed / COTS metadata this repo adds. New parts are covered the moment they land.
"""
import importlib
import pkgutil

import pytest

import parts
from lib import reference as R

PART_NAMES = sorted(mi.name for mi in pkgutil.iter_modules(parts.__path__) if not mi.name.startswith("_"))

# Parts that are DELIBERATELY several disjoint solids (multi-body SolidWorks parts / vendor
# files): name -> expected solid count. Everything else must be exactly one solid.
MULTI_BODY = {
    "gripper_j3_connector": 2,
    "gt2_pulley_20t": 3,
    "mg996r_servo": 4,
    "nema17_pancake": 11,
}


def test_some_parts_were_discovered():
    assert PART_NAMES, "no parts discovered under parts/"


def test_naming_map_matches_part_files():
    expected = set(R.CUSTOM) | set(R.COTS) | set(R.DESIGNED)
    assert set(PART_NAMES) == expected, (
        f"parts/ and lib/reference.py disagree: only in parts/: {set(PART_NAMES) - expected}; "
        f"only in the map: {expected - set(PART_NAMES)}"
    )


@pytest.mark.parametrize("name", PART_NAMES)
def test_part_declares_its_contract(name):
    """Fast: metadata only, no geometry."""
    mod = importlib.import_module(f"parts.{name}")
    assert callable(getattr(mod, "gen_step", None)), f"parts.{name} must expose a callable gen_step()"
    if getattr(mod, "COTS", False):
        assert name in R.COTS, f"{name} declares COTS but is not in lib.reference.COTS"
        mass = getattr(mod, "MASS_G", None)
        assert isinstance(mass, (int, float)) and mass > 0, f"COTS part {name} must set MASS_G > 0"
        assert mod.VENDOR_STEP == R.VENDOR_DIR / f"{name}.step", f"{name}.VENDOR_STEP must be vendor/{name}.step"
    else:
        assert name in R.CUSTOM or name in R.DESIGNED, f"{name} is not in lib.reference.CUSTOM / DESIGNED (and not COTS)"
        assert getattr(mod, "REFERENCE", None) == name, f"{name}.REFERENCE must name reference/{name}.step"
        assert isinstance(getattr(mod, "CONVERTED", None), bool), f"{name} must declare CONVERTED = True/False"
        assert R.path_of(mod.REFERENCE).exists(), f"missing reference/{name}.step (run tools/import_reference.py)"


@pytest.mark.slow
@pytest.mark.parametrize("name", PART_NAMES)
def test_part_builds_valid_labelled_geometry(name):
    mod = importlib.import_module(f"parts.{name}")
    shape = mod.gen_step()
    assert shape is not None, f"{name}.gen_step() returned None"
    assert shape.label == name, f"{name} must label its result with its own name, got {shape.label!r}"
    assert shape.is_valid, f"{name} built an invalid shape"
    assert R.solid_volume(shape) > 0, f"{name} built an empty/zero-volume shape"
    got, expected = len(shape.solids()), MULTI_BODY.get(name, 1)
    assert got == expected, (
        f"{name} built {got} disjoint solids, expected {expected}. A stray solid usually means a "
        f"cutter helper leaked geometry into the active BuildPart; if this part is intentionally "
        f"multi-body, register it in MULTI_BODY."
    )


COTS_PARTS = [n for n in PART_NAMES if n in R.COTS]


@pytest.mark.parametrize("name", COTS_PARTS)
def test_cots_envelope_tracks_reference_bbox(name):
    """The fallback envelope must occupy the vendor geometry's bounding box (same frame)."""
    import json

    mod = importlib.import_module(f"parts.{name}")
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
    mod = importlib.import_module(f"parts.{name}")
    if not mod.VENDOR_STEP.exists():
        pytest.skip(f"no vendor/{name}.step - envelope in use")
    shape = mod.gen_step()
    ref = R.load(name)
    for got, exp, what in (
        (R.bbox_min(shape), R.bbox_min(ref), "bbox min"),
        (R.bbox_size(shape), R.bbox_size(ref), "bbox size"),
    ):
        assert all(abs(g - e) <= COTS_FRAME_TOL_MM for g, e in zip(got, exp)), (
            f"{name} vendor geometry {what} {got} vs reference {exp} (tol {COTS_FRAME_TOL_MM} mm) - "
            f"set VENDOR_TO_REF in parts/{name}.py"
        )
