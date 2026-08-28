"""Copy the SolidWorks per-part STEP exports into reference/ under clean snake_case names
(every part, custom and purchased - the immutable frame/size reference), seed vendor/ for
purchased parts, and write reference/manifest.json. The cycloidal drive's parts are not
SolidWorks exports: tools/import_cycloidal_reference.py owns them (their entries are kept).

    ./cadtool python tools/import_reference.py [--src DIR] [--force]

Copies go through the explicit map in lib/reference.py - never shell globs: the source
names contain spaces, parentheses, a trailing space and a Cyrillic configuration name.
reference/*.step are immutable inputs; vendor/*.step may be replaced by better models (see
vendor/README.md); parts/*.step are regenerated from Python. Re-run (without --force) after
tools/extract_placements.py and after replacing a vendor file, so the manifest is current.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from lib import reference as R  # noqa: E402

sha256, step_units, describe = R.sha256, R.step_units, R.describe

MANIFEST_PATH = R.REF_DIR / "manifest.json"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=pathlib.Path,
                    default=pathlib.Path(os.environ.get("ARM_REFERENCE_SRC", R.DEFAULT_SOURCE_DIR)),
                    help="SolidWorks export tree (default: %(default)s)")
    ap.add_argument("--force", action="store_true", help="overwrite existing copies")
    args = ap.parse_args(argv)
    if not args.src.is_dir():
        print(f"source dir not found: {args.src}", file=sys.stderr)
        return 1

    R.REF_DIR.mkdir(exist_ok=True)
    R.VENDOR_DIR.mkdir(exist_ok=True)
    jobs = [(name, prod, rel, "custom") for name, (prod, rel) in R.CUSTOM.items()]
    jobs += [(name, prod, rel, "cots") for name, (prod, rel) in R.COTS.items() if name not in R.CYCLOIDAL_PARTS]

    # The cycloidal drive's entries are owned by tools/import_cycloidal_reference.py - keep them.
    existing = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))["parts"] if MANIFEST_PATH.exists() else {}
    manifest = {
        "source_dir": str(args.src),
        "monolith": None,
        "parts": {name: entry for name, entry in existing.items() if name in R.CYCLOIDAL_PARTS},
    }
    monolith = args.src / R.MONOLITH_NAME
    if monolith.exists():
        manifest["monolith"] = {"file": R.MONOLITH_NAME, "bytes": monolith.stat().st_size, "sha256": sha256(monolith)}

    missing: list[str] = []
    for name, prod, rel, kind in jobs:
        ref = R.REF_DIR / f"{name}.step"          # immutable SolidWorks geometry (frame + size reference)
        vendor = R.VENDOR_DIR / f"{name}.step"    # COTS only: the current best vendor model (may be replaced)
        if rel is None:
            # Extracted from the full assembly by tools/extract_placements.py into vendor/; that
            # extraction IS the SolidWorks reference, so mirror it into reference/ once.
            if not vendor.exists():
                print(f"{name:24s} {kind:7s} (not yet extracted - run tools/extract_placements.py)")
                continue
            if not ref.exists() or args.force:
                shutil.copyfile(vendor, ref)
            action = "extracted"
        else:
            src = args.src / rel
            if not src.exists():
                missing.append(f"{name}: {src}")
                continue
            if ref.exists() and not args.force:
                action = "kept"
            else:
                shutil.copyfile(src, ref)
                action = "copied"
            if kind == "cots" and not vendor.exists():
                shutil.copyfile(src, vendor)   # seed the vendor model; step.parts downloads replace it
        entry = {
            "kind": kind,
            "product": prod,
            "source": rel,
            "bytes": ref.stat().st_size,
            "sha256": sha256(ref),
            "units": step_units(ref),
            **describe(ref),
        }
        if kind == "cots":
            entry["vendor"] = {
                "bytes": vendor.stat().st_size,
                "sha256": sha256(vendor),
                "same_as_reference": sha256(vendor) == entry["sha256"],
                **describe(vendor),
            }
        manifest["parts"][name] = entry
        vend = "" if kind != "cots" else ("  vendor=reference" if entry["vendor"]["same_as_reference"] else "  vendor=REPLACED")
        print(f"{name:24s} {kind:7s} {action:9s} {entry['units']:4s} solids={entry['solids']:2d} "
              f"vol={entry['solid_volume']:12.1f}  size={entry['bbox_size']}{vend}")

    MANIFEST_PATH.write_text(json.dumps(manifest, indent=1, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {MANIFEST_PATH} ({len(manifest['parts'])} entries)")
    if missing:
        print("MISSING sources:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
