"""Copy the CadQuery exports of the cycloidal drive (tools/export_cycloidal_cadquery.py, run in
the cycloidal_drive repo) into reference/<name>.step and merge their entries into
reference/manifest.json (kind "designed" for the printed parts, "cots" for the purchased ones).

    ./cadtool python tools/import_cycloidal_reference.py [--src ../cycloidal_drive/export/step/house]
                                                          [--force] [--only NAME ...]

Owns exactly the manifest entries of lib/reference.py DESIGNED | CYCLOIDAL_COTS;
tools/import_reference.py owns the SolidWorks ones and keeps these untouched. Re-run after
replacing a vendor/<name>.step (the entry's `vendor` block records the current file).
"""
from __future__ import annotations

import argparse
import json
import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from lib import reference as R  # noqa: E402

MANIFEST_PATH = R.REF_DIR / "manifest.json"
DEFAULT_SRC = R.CAD_DIR.parent.parent / "cycloidal_drive" / "export" / "step" / "house"

# Human-readable product names for the manifest / reference README.
PRODUCTS = {
    "cycloidal_disc_1": "cycloidal disc 1 (20 lobes, 0 deg phase)",
    "cycloidal_disc_2": "cycloidal disc 2 (20 lobes, -9 deg phase)",
    "cycloidal_eccentric_shaft": "eccentric shaft (two 17.1 mm lobes, D-bore input)",
    "cycloidal_motor_plate": "motor plate (NEMA 17 side housing half)",
    "cycloidal_ring_gear_body": "ring gear body (housing, 6814 seat, nut pockets)",
    "cycloidal_output_hub": "output hub (arm mount, 4x M4 on 50 mm)",
    "bearing_6003": "6003-2RS deep-groove ball bearing 17x35x10",
    "bearing_6814": "6814-2RS deep-groove ball bearing 70x90x10",
    "bearing_625": "625-2RS deep-groove ball bearing 5x16x5",
    "nema17_48mm": "NEMA 17 stepper, 48 mm body, 5 mm D shaft",
    "cycloidal_ring_pins": "21x 4x35 mm h6 dowel pins (ring pins)",
    "cycloidal_output_pins": "4x 4x45 mm h6 dowel pins (output pins)",
    "cycloidal_shaft_support_pin": "5x20 mm h6 dowel pin (eccentric-shaft support)",
    "cycloidal_motor_bolts": "4x M3x10 SHCS (motor)",
    "cycloidal_housing_bolts": "8x M4x55 SHCS (housing)",
    "cycloidal_housing_nuts": "8x M4 hex nuts (housing)",
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=pathlib.Path, default=DEFAULT_SRC, help="exporter output dir (default: %(default)s)")
    ap.add_argument("--force", action="store_true", help="overwrite existing reference copies")
    ap.add_argument("--only", nargs="*", default=None, help="subset of names")
    args = ap.parse_args(argv)
    if not args.src.is_dir():
        print(f"source dir not found: {args.src} - run tools/export_cycloidal_cadquery.py in the cycloidal_drive repo", file=sys.stderr)
        return 1
    src_manifest = json.loads((args.src / "manifest.json").read_text()) if (args.src / "manifest.json").exists() else {}
    rev = src_manifest.get("rev", R.CYCLOIDAL_REV)
    if rev != R.CYCLOIDAL_REV or src_manifest.get("dirty"):
        print(f"WARNING: exports are from rev {rev}{' (dirty)' if src_manifest.get('dirty') else ''}, "
              f"lib/reference.py says {R.CYCLOIDAL_REV}", file=sys.stderr)

    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8")) if MANIFEST_PATH.exists() else {"parts": {}}
    jobs = [(n, "designed", b) for n, b in R.DESIGNED.items()] + [(n, "cots", b) for n, b in R.CYCLOIDAL_COTS.items()]
    missing = []
    for name, kind, builder in jobs:
        if args.only and name not in args.only:
            continue
        src = args.src / f"{name}.step"
        if not src.exists():
            missing.append(f"{name}: {src}")
            continue
        ref = R.path_of(name)
        if ref.exists() and not args.force:
            action = "kept"
        else:
            shutil.copyfile(src, ref)
            action = "copied"
        entry = {
            "kind": kind, "origin": f"cycloidal_drive@{rev}", "product": PRODUCTS.get(name, name),
            "source": builder, "bytes": ref.stat().st_size, "sha256": R.sha256(ref), "units": R.step_units(ref),
            **R.describe(ref),
        }
        vendor = R.VENDOR_DIR / f"{name}.step"
        if kind == "cots" and vendor.exists():
            entry["vendor"] = {
                "bytes": vendor.stat().st_size, "sha256": R.sha256(vendor),
                "same_as_reference": R.sha256(vendor) == entry["sha256"], **R.describe(vendor),
            }
        manifest["parts"][name] = entry
        vend = "" if kind != "cots" else ("  vendor=" + ("reference" if entry.get("vendor", {}).get("same_as_reference") else "step.parts" if "vendor" in entry else "envelope"))
        print(f"{name:28s} {kind:8s} {action:7s} solids={entry['solids']:2d} vol={entry['solid_volume']:12.1f}  size={entry['bbox_size']}{vend}")

    MANIFEST_PATH.write_text(json.dumps(manifest, indent=1, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {MANIFEST_PATH} ({len(manifest['parts'])} entries)")
    if missing:
        print("MISSING exports:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
