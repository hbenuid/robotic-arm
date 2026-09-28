"""The `@step` model convention, enforced automatically for every parts/<group>/<name>.py.

Auto-discovers parts exactly like the assemblies do (parts.names(): every module under a group
package that does not start with '_') and checks each honours the cadgen `@step` contract: importable
with no side effects, ONE `@step def <name>()` model named after the file whose STEP is the
sibling file (no out=), whose body (parts.build(name) - never the model itself, that would start a
build) returns a valid, non-empty, labelled shape, plus the wrapper / designed / COTS metadata this
repo adds. New parts are covered the moment they land.
"""
import pytest

import parts
from lib import manifest as M
from lib import reference as R
from lib.datum import IDENTITY
from tests import built
from tests.source_checks import runs_its_model

PART_NAMES = parts.names()

# Parts that are DELIBERATELY several disjoint solids (multi-body SolidWorks parts / vendor
# files): name -> expected solid count. Everything else must be exactly one solid.
MULTI_BODY = {
    "gripper_j3_connector": 2,
    "gt2_pulley_20t": 3,
    "mg996r_servo": 4,
    "nema17_pancake": 11,
    "nema17_40mm": 2,       # body + D-shaft (the kit export split by tools/reference/split_mks_motor.py)
    "mks_servo42d": 13,     # PCB (4) + cover + 4 standoffs + 4 M3x30 - the kit's board half, one piece per occurrence
    "nema17_48mm": 7,       # the drive motor: front plate, housing, back plate, 2 bearings, connector, rotor + shaft (composed by the same tool)
    # cycloidal drive fastener / pin patterns (one compound per pattern)
    "cycloidal_ring_pins": 21,
    "cycloidal_output_pins": 4,
    "cycloidal_motor_bolts": 4,
    "cycloidal_housing_bolts": 6,
    "cycloidal_housing_nuts": 6,
    # the 90T pulley bolts of the elbow and the wrist (lib/mounts.py FASTENER_MOUNTS)
    "elbow_pulley_screws": 4,
    "elbow_pulley_nuts": 4,
    "wrist_pulley_screws": 4,
    "wrist_pulley_nuts": 4,
    # the roll motor mount's countersunk screws + nuts (assemblies/forearm_roll_drive.py rows)
    "forearm_roll_mount_screws": 4,
    "forearm_roll_mount_nuts": 4,
    # the base motor mount's M4 screws + nuts (lib/mounts.py BASE_MOUNTS)
    "base_motor_mount_screws": 4,
    "base_motor_mount_nuts": 4,
}


def test_some_parts_were_discovered():
    assert PART_NAMES, "no parts discovered under parts/"


def test_naming_map_matches_part_files():
    expected = set(R.CUSTOM) | set(R.COTS) | set(R.DESIGNED) | set(R.NATIVE)
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
        spec, qty = getattr(mod, "PURCHASE_SPEC", None), getattr(mod, "PURCHASE_QTY", None)
        assert isinstance(spec, str) and spec.strip(), f"COTS part {name} must set PURCHASE_SPEC (what to order - tools/bom.py)"
        assert isinstance(qty, int) and qty >= 1, f"COTS part {name} must set PURCHASE_QTY (pieces per occurrence)"
        assert isinstance(getattr(mod, "PURCHASE_NOTE", ""), str)
        if name in MULTI_BODY and (name.startswith("cycloidal_") or name.endswith(("_screws", "_nuts"))):
            # a fastener / pin pattern: one piece per solid (not a multi-body vendor motor)
            assert qty == MULTI_BODY[name], f"{name}.PURCHASE_QTY {qty} != its {MULTI_BODY[name]} solids"
    else:
        assert not any(hasattr(mod, a) for a in ("PURCHASE_SPEC", "PURCHASE_QTY", "PURCHASE_NOTE")), (
            f"{name} is printed (no COTS = True) but declares PURCHASE_* - it would never reach the buy list")
        assert name in R.CUSTOM or name in R.DESIGNED or name in R.NATIVE, f"{name} is not in lib.reference.CUSTOM / DESIGNED / NATIVE (and not COTS)"
        assert getattr(mod, "REFERENCE", None) == name, f"{name}.REFERENCE must name reference/<origin>/{name}.step"
        assert isinstance(getattr(mod, "CONVERTED", None), bool), f"{name} must declare CONVERTED = True/False"
        if name in R.NATIVE:
            assert mod.CONVERTED is True, f"{name} is native build123d - CONVERTED must be True"
        assert R.path_of(mod.REFERENCE).exists(), (
            f"missing {R.path_of(name)} (run tools/reference/import_solidworks.py, tools/cycloidal/import_cadquery.py "
            f"or, for a native part, tools/reference/import_native.py)")
        if hasattr(mod, "REFERENCE_BUILD"):
            assert mod.CONVERTED is True and callable(mod.REFERENCE_BUILD), f"{name}.REFERENCE_BUILD is for a converted part's LEGACY build"


@pytest.mark.slow
@pytest.mark.parametrize("name", PART_NAMES)
def test_part_builds_valid_labelled_geometry(name):
    shape = built.part(name)
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

    shape = built.part(name)
    back = cp._build123d_shape_from_brep_bytes(cp._shape_brep_bytes(shape))
    assert back.is_valid and len(back.solids()) == len(shape.solids())
    assert abs(R.solid_volume(back) - R.solid_volume(shape)) < 1e-6 * max(1.0, R.solid_volume(shape))


COTS_PARTS = [n for n in PART_NAMES if n in R.COTS]


@pytest.mark.parametrize("name", COTS_PARTS)
def test_cots_envelope_tracks_reference_bbox(name):
    """The fallback envelope must occupy the vendor geometry's bounding box (same frame) - the REFERENCE_BUILD's,
    for a part that has left its reference (the drive's housing fasteners: the port's pattern)."""
    mod = parts.load(name)
    entry = M.read()["parts"][name]
    env = getattr(mod, "REFERENCE_BUILD", mod._envelope)()
    assert env.is_valid
    for got, exp in ((R.bbox_min(env), entry["bbox_min"]), (R.bbox_size(env), entry["bbox_size"])):
        assert all(abs(g - e) <= 0.05 for g, e in zip(got, exp, strict=True)), f"{name} envelope {got} vs reference {exp}"


COTS_FRAME_TOL_MM = 1.5   # catalog models differ slightly from the SolidWorks re-exports


def _vendor_differs(name: str) -> bool:
    """A vendor file the frame check can tell from the reference: one that is not the reference file itself
    (manifest same_as_reference), or one moved by VENDOR_TO_REF. With no vendor file the envelope is the geometry
    (test_cots_envelope_tracks_reference_bbox)."""
    vendor = M.read()["parts"][name].get("vendor")
    return vendor is not None and (not vendor["same_as_reference"] or parts.load(name).VENDOR_TO_REF != IDENTITY)


VENDOR_PARTS = [n for n in COTS_PARTS if _vendor_differs(n)]


@pytest.mark.slow
@pytest.mark.parametrize("name", VENDOR_PARTS)
def test_cots_vendor_matches_reference_frame(name):
    """The vendor geometry (after VENDOR_TO_REF) must occupy the SolidWorks reference's bounding
    box - guards the re-orientation of a swapped-in step.parts model."""
    shape = built.part(name)
    ref = R.load(name)
    for got, exp, what in (
        (R.bbox_min(shape), R.bbox_min(ref), "bbox min"),
        (R.bbox_size(shape), R.bbox_size(ref), "bbox size"),
    ):
        assert all(abs(g - e) <= COTS_FRAME_TOL_MM for g, e in zip(got, exp, strict=True)), (
            f"{name} vendor geometry {what} {got} vs reference {exp} (tol {COTS_FRAME_TOL_MM} mm) - "
            f"set VENDOR_TO_REF in parts/{name}.py"
        )
