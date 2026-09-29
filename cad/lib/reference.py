"""SolidWorks reference geometry: the naming map, loaders and the reference-match check.

The originals live outside the repo (~/Documents/arm_assembly_organized/, SolidWorks 2026
AP214 exports of 2026-08-27). `tools/reference/import_solidworks.py` copies the per-part exports into
reference/solidworks/<clean_name>.step (immutable inputs) and vendor/<clean_name>.step (purchased
parts); `tools/reference/extract_placements.py` extracts the assembly placements from the
full-assembly STEP into reference/placements.json (and the motor mounts declared in lib/mounts.py,
tools/reference/mount_placements.py). The cycloidal drive's references are CadQuery exports in
reference/cycloidal/ (tools/cycloidal/import_cadquery.py); the parts designed in this repo (NATIVE) keep
their accepted build in reference/native/ (tools/reference/import_native.py); path_of() resolves the
origin. The third producer of vendor files is tools/reference/split_mks_motor.py: it splits the "NEMA 17
x 40 + MKS SERVO42D" kit export (MKS_EXPORT_NAME, next to the monolith) into vendor/nema17_40mm.step and
vendor/mks_servo42d.step, which import_solidworks.py then mirrors into reference/solidworks/.

`lib/` never imports `parts/`.
"""
from __future__ import annotations

import hashlib
import pathlib
import unicodedata

from cadgen import build123d as bd
from cadgen import read_step

CAD_DIR = pathlib.Path(__file__).resolve().parent.parent
REF_DIR = CAD_DIR / "reference"              # manifest.json, placements.json + the two origin dirs
REF_SOLIDWORKS_DIR = REF_DIR / "solidworks"  # the SolidWorks per-part exports (CUSTOM + the SolidWorks COTS)
REF_CYCLOIDAL_DIR = REF_DIR / "cycloidal"    # the CadQuery exports of the drive (DESIGNED | CYCLOIDAL_COTS)
REF_NATIVE_DIR = REF_DIR / "native"          # accepted builds of the parts designed HERE (NATIVE | NATIVE_COTS)
VENDOR_DIR = CAD_DIR / "vendor"

# Where the SolidWorks export tree lives on this machine (override: --src / ARM_REFERENCE_SRC).
DEFAULT_SOURCE_DIR = pathlib.Path.home() / "Documents" / "arm_assembly_organized"
MONOLITH_NAME = "final Arm Assembly Fully Movable.STEP"   # 13 MB, inch units, the full positioned assembly
MKS_EXPORT_NAME = "mks/nema17x40_with_mks.step"           # 0.5 MB, mm, SolidWorks 2026 export of the NEMA 17 x 40 + MKS SERVO42D kit
#   (sha256 4e51a159...; tools/reference/split_mks_motor.py splits it into vendor/nema17_40mm.step + vendor/mks_servo42d.step)
MKS48_EXPORT_NAME = "mks/nema17x48_with_mks.step"         # 3.3 MB, cm, the same kit with a 48 mm motor (a 17HS19-2004S1: 24 mm shaft, 15 mm D-cut)
#   (sha256 c1958e60...; the same tool composes vendor/nema17_48mm.step from its body + the x40's shaft trimmed to 22)

# Custom / printed parts: clean name -> (SolidWorks product name, export path under the source dir).
# Each gets parts/<group>/<name>.py (an import wrapper until converted) + reference/solidworks/<name>.step.
CUSTOM: dict[str, tuple[str, str]] = {
    "base":                  ("base of robot arm 62126",                 "step/base of robot arm 62126.STEP"),
    "j1_coupler":            ("Base couple updated 62126 _J1 coupler",   "step/Base couple updated 62126 _J1 coupler.STEP"),
    "j1_link":               ("first joint edit 62126",                  "step/first joint edit 62126.STEP"),
    "j2_link":               ("Joint 2 change 8126",                     "step/Joint 2 change 8126.STEP"),
    "j3_coupler":            ("Joint 2 coupler 62226_J3 Coupler",        "step/Joint 2 coupler 62226_J3 Coupler.STEP"),
    "gt2_pulley_90t":        ("GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric",
                              "step/GT2 Pulley - 90 teeth - J1 - 62226_GT2 Pulley - Parametric.STEP"),
    "gripper_clamp_bracket": ("brack for hand cmap",                     "step/brack for hand cmap.STEP"),
    "wrist_link":            ("final component arm qwrist movement",     "step/final component arm qwrist movement.STEP"),
    "gripper_cover":         ("Gripper Cover_Gripper Cover",             "step/Gripper Cover_Gripper Cover.STEP"),
    "gripper_end":           ("Gripper End_Gripper End",                 "step/Gripper End_Gripper End.STEP"),
    "gripper_finger_left":   ("Gripper Hand Left_Gripper Hand Left",     "step/Gripper Hand Left_Gripper Hand Left.STEP"),
    "gripper_finger_right":  ("Gripper Hand Right_Gripper Hand Left",    "step/Gripper Hand Right_Gripper Hand Left.STEP"),  # mirror of Left; stale config name
    "gripper_link_1":        ("Gripper link 1_Gripper link 1",           "step/Gripper link 1_Gripper link 1.STEP"),
    "gripper_link_2":        ("Gripper link 2_Gripper link 2",           "step/Gripper link 2_Gripper link 2.STEP"),
    "gripper_slider":        ("Gripper Mechanism Slider_Gripper Mechanism Slider",
                              "step/Gripper Mechanism Slider_Gripper Mechanism Slider.STEP"),
    "gripper_j3_connector":  ("Gripper to J3 connector 7726_Gripper to J3 connector",
                              "step/Gripper to J3 connector 7726_Gripper to J3 connector.STEP"),
    "servo_holder":          ("Servo Holder_Servo Holder",               "step/Servo Holder_Servo Holder.STEP"),
}

# Purchased (COTS) parts: clean name -> (product name, export path, or None when extracted
# from the full assembly). Each gets parts/<group>/<name>.py (COTS = True) + vendor/<name>.step.
COTS: dict[str, tuple[str, str | None]] = {
    "gt2_pulley_20t":   ("GT2_20T_Конфигурация1",                                    "step/GT2_20T_Конфигурация1.STEP"),
    "gripper_rail_6mm": ("Gripper rail 6mm_Gripper rail 6mm",                        "step/Gripper rail 6mm_Gripper rail 6mm.STEP"),
    "mg996r_servo":     ("Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model",  "step/Servo Motor MG996R 3D Model_Servo Motor MG996R 3D Model.STEP"),
    "mg996r_horn":      ("Servo MG996R Horn_Servo MG996R Horn",                      "step/Servo MG996R Horn_Servo MG996R Horn.STEP"),
    "nema17_pancake":   ("nema17_pancake", None),   # 7-part sub-assembly, flattened by tools/reference/extract_placements.py
    # The NEMA 17 x 40 + MKS SERVO42D kit export (MKS_EXPORT_NAME), split by tools/reference/split_mks_motor.py:
    "nema17_40mm":      ("nema17x40_with_mks: motor body + D-shaft (shaft trimmed to the drive motor's 22 mm)", None),
    "mks_servo42d":     ("nema17x40_with_mks: Servo42D_Assem (PCB + cover) + 4 standoffs + 4 M3x30", None),
}

# Sub-assemblies kept as modules under assemblies/: clean name -> product name.
MODULES: dict[str, str] = {
    "gripper": "Gripper Mechanism_Gripper Mechanism",
}

# Designed (parametric build123d) parts ported from the cycloidal_drive repo. Their reference
# is NOT a SolidWorks export but the CadQuery builder's own STEP export at CYCLOIDAL_REV
# (reference/cycloidal/<name>.step, manifest kind "designed"; tools/cycloidal/export_cadquery.py +
# tools/cycloidal/import_cadquery.py). Values: the builder label the exporter uses.
CYCLOIDAL_REV = "2f1f67d"
DESIGNED: dict[str, str] = {
    "cycloidal_disc_1":          "src/cycloidal_disc.py:build_cycloidal_disc()",
    "cycloidal_disc_2":          "src/cycloidal_disc.py:build_cycloidal_disc(phase_offset_deg=disc2_phase)",
    "cycloidal_eccentric_shaft": "src/eccentric_shaft.py:build_eccentric_shaft()",
    "cycloidal_motor_plate":     "src/motor_plate.py:build_motor_plate()",
    "cycloidal_ring_gear_body":  "src/ring_gear_body.py:build_ring_gear_body()",
    "cycloidal_output_hub":      "src/output_hub.py:build_output_hub()",
}

# Purchased parts of the cycloidal drive: clean name -> CadQuery builder of the simplified model
# (its export is reference/cycloidal/<name>.step; a step.parts model may live in vendor/<name>.step).
CYCLOIDAL_COTS: dict[str, str] = {
    "bearing_6003":                "src/purchased_parts.py:build_bearing_6003()",
    "bearing_6814":                "src/purchased_parts.py:build_bearing_6814()",
    "bearing_625":                 "src/purchased_parts.py:build_bearing_625()",
    "nema17_48mm":                 "src/purchased_parts.py:build_nema17_motor()",
    "cycloidal_ring_pins":         "src/purchased_parts.py:build_ring_pins()",
    "cycloidal_output_pins":       "src/purchased_parts.py:build_output_pins()",
    "cycloidal_shaft_support_pin": "src/purchased_parts.py:build_shaft_support_pin()",
    "cycloidal_motor_bolts":       "src/purchased_parts.py:build_motor_bolts()",
    "cycloidal_housing_bolts":     "src/purchased_parts.py:build_housing_bolts()",
    "cycloidal_housing_nuts":      "src/purchased_parts.py:build_housing_nuts()",
}
COTS.update({name: (f"cycloidal_drive {builder}", None) for name, builder in CYCLOIDAL_COTS.items()})
CYCLOIDAL_PARTS: set[str] = set(DESIGNED) | set(CYCLOIDAL_COTS)

# Native parts: designed in this repo in build123d, with no SolidWorks or CadQuery origin. Their reference
# is their own ACCEPTED build - reference/native/<name>.step, written once by
# tools/reference/import_native.py (manifest kind "native"; re-run it to accept a changed design) - so
# tests/test_reference_match.py locks their geometry like every other part's. Values: the builder label.
NATIVE: dict[str, str] = {
    "forearm_roll_block":    "lib/forearm/roll.py:build_block(DEFAULT)",
    "forearm_roll_shaft":    "lib/forearm/roll.py:build_shaft(DEFAULT)",
    "forearm_roll_retainer": "lib/forearm/roll.py:build_retainer(DEFAULT)",
    "forearm_roll_motor_mount": "lib/forearm/roll.py:build_motor_mount(DEFAULT)",
    "base_motor_mount":      "lib/base/body.py:build_motor_mount(DEFAULT)",
}

# Purchased parts with neither a SolidWorks export nor a catalog model (their envelope IS the geometry):
# clean name -> builder label. The same tool writes their reference (kind "cots") from the envelope.
NATIVE_COTS: dict[str, str] = {
    "bearing_6808": "parts/joints/bearing_6808.py:_envelope()",
    "bearing_6806": "parts/joints/bearing_6806.py:_envelope()",
    "gt2_idler_20t": "parts/joints/gt2_idler_20t.py:_envelope()",
    "bearing_axk6590": "parts/base/bearing_axk6590.py:_envelope()",
    "washer_as6590": "parts/base/washer_as6590.py:_envelope()",
    "elbow_pulley_screws": "parts/joints/elbow_pulley_screws.py:_envelope()",
    "elbow_pulley_nuts": "parts/joints/elbow_pulley_nuts.py:_envelope()",
    "wrist_pulley_screws": "parts/joints/wrist_pulley_screws.py:_envelope()",
    "wrist_pulley_nuts": "parts/joints/wrist_pulley_nuts.py:_envelope()",
    "forearm_roll_mount_screws": "parts/joints/forearm_roll_mount_screws.py:_envelope()",
    "forearm_roll_mount_nuts": "parts/joints/forearm_roll_mount_nuts.py:_envelope()",
    "base_motor_mount_screws": "parts/base/base_motor_mount_screws.py:_envelope()",
    "base_motor_mount_nuts": "parts/base/base_motor_mount_nuts.py:_envelope()",
    "ky003_hall_sensor": "parts/joints/ky003_hall_sensor.py:_envelope()",
}
COTS.update({name: (f"native {builder}", None) for name, builder in NATIVE_COTS.items()})
NATIVE_PARTS: set[str] = set(NATIVE) | set(NATIVE_COTS)

# Sub-assemblies whose contents are code-driven (assemblies/<name>.py places its parts from a lib/ stack):
# clean name -> product name of the SolidWorks node that places it, or a description when the pose is
# declared in lib/mounts.py MODULE_MOUNTS instead (tools/reference/mount_placements.py writes the record).
DESIGNED_MODULES: dict[str, str] = {
    "cycloidal_drive": "New cyloidal assembly",   # sic - the SolidWorks node is misspelled
    "forearm_roll_drive": "forearm roll drive (no SolidWorks node: lib/mounts.py MODULE_MOUNTS places it on j2_link#1)",
}

# Full-assembly nodes deliberately not modelled here (whole subtree skipped; a top-level one lands in placements.json
# `skipped` with its pose - tools/reference/extract_placements.py, or mount_placements.py's merge mode for a record
# that was an occurrence before).
SKIPPED_PRODUCTS: dict[str, str] = {
    "nema17_pancake(2)":        "pancake internals are flattened into vendor/nema17_pancake.step",
    "cap 1 joint 2 8726":       "j2_cap_1, the lid over j2_link's motor side - the link caps were removed 2026-09-25",
    "cap of joint 2 piece 2 8526": "j2_cap_2, the belt tray under j2_link - the link caps were removed 2026-09-25",
    "first joint cap 8726":     "j1_cap, the tray under j1_link - the link caps were removed 2026-09-25",
}

PRODUCT_TO_PART: dict[str, str] = {
    prod: name for name, (prod, _) in {**CUSTOM, **COTS}.items() if name not in CYCLOIDAL_PARTS | NATIVE_PARTS
}


def clean_label(product_name: str) -> str:
    """Mangle a STEP product name the way build123d.import_step() labels nodes
    (control characters stripped, ' .()' -> '_'), so node labels map back to products."""
    text = "".join(ch for ch in product_name if unicodedata.category(ch)[0] != "C")
    return text.translate(str.maketrans(" .()", "____"))


LABEL_TO_PART: dict[str, str] = {clean_label(p): n for p, n in PRODUCT_TO_PART.items()}
LABEL_TO_MODULE: dict[str, str] = {clean_label(p): n for n, p in MODULES.items()}
SKIPPED_LABELS: dict[str, str] = {clean_label(p): why for p, why in SKIPPED_PRODUCTS.items()}
LABEL_TO_DESIGNED_MODULE: dict[str, str] = {clean_label(p): n for n, p in DESIGNED_MODULES.items()}


def reference_dir(name: str) -> pathlib.Path:
    """Origin directory of reference/<origin>/<name>.step."""
    if name in CYCLOIDAL_PARTS:
        return REF_CYCLOIDAL_DIR
    return REF_NATIVE_DIR if name in NATIVE_PARTS else REF_SOLIDWORKS_DIR


def path_of(name: str) -> pathlib.Path:
    return reference_dir(name) / f"{name}.step"


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def step_units(path: pathlib.Path) -> str:
    data = path.read_bytes()
    return "inch" if (b"CONVERSION_BASED_UNIT" in data and b"'INCH'" in data) else "mm"


def describe(path: pathlib.Path) -> dict:
    """solids / solid_volume / bbox of a STEP file - the manifest.json geometry facts."""
    shape = bd.import_step(str(path))
    bb = shape.bounding_box()
    return {
        "solids": len(shape.solids()),
        "solid_volume": round(solid_volume(shape), 3),
        "bbox_min": [round(v, 3) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
        "bbox_size": [round(v, 3) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
    }


def load(name: str, *, label: str | None = None) -> bd.Shape:
    """Fresh read of reference/<origin>/<name>.step in its part-file frame (cadgen.read_step:
    inside a build the file joins the model's closure, so an updated reference makes it stale).
    A Solid for one-body parts, a flat Compound for multi-body ones."""
    path = path_of(name)
    if not path.exists():
        raise FileNotFoundError(f"missing reference STEP {path} - run tools/reference/import_solidworks.py "
                                f"(tools/cycloidal/import_cadquery.py for the drive, tools/reference/import_native.py for a native part)")
    shape = read_step(path)          # cadgen: store-cached; a tracked input of the model that calls it
    shape.label = label or name
    return shape


def solid_volume(shape: bd.Shape) -> float:
    """Sum of solid volumes. Never use Compound.volume: in build123d 0.10 it skips nested
    sub-assembly compounds."""
    return sum(s.volume for s in shape.solids())


def bbox_size(shape: bd.Shape) -> tuple[float, float, float]:
    bb = shape.bounding_box()
    return (bb.size.X, bb.size.Y, bb.size.Z)


def bbox_min(shape: bd.Shape) -> tuple[float, float, float]:
    bb = shape.bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z)


def matches_reference(
    shape: bd.Shape,
    name: str,
    *,
    local_from_ref: bd.Location | None = None,
    vol_tol: float = 0.005,
    bbox_tol: float = 0.2,
    check_position: bool = True,
) -> tuple[bool, dict]:
    """Compare `shape` (in its part-local frame) with reference/<origin>/<name>.step moved by
    `local_from_ref`. Volume must agree within `vol_tol` (relative); bounding-box size and,
    if `check_position`, bounding-box min within `bbox_tol` mm. Returns (ok, report)."""
    ref = load(name).moved(local_from_ref or bd.Location())
    vol, ref_vol = solid_volume(shape), solid_volume(ref)
    size, ref_size = bbox_size(shape), bbox_size(ref)
    lo, ref_lo = bbox_min(shape), bbox_min(ref)
    report = {
        "volume": (vol, ref_vol),
        "bbox_size": (size, ref_size),
        "bbox_min": (lo, ref_lo),
        "solids": (len(shape.solids()), len(ref.solids())),
    }
    ok = abs(vol - ref_vol) <= vol_tol * ref_vol
    ok = ok and all(abs(a - b) <= bbox_tol for a, b in zip(size, ref_size, strict=True))
    if check_position:
        ok = ok and all(abs(a - b) <= bbox_tol for a, b in zip(lo, ref_lo, strict=True))
    return ok, report
