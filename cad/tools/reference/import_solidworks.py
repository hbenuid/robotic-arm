"""Copy the SolidWorks per-part STEP exports into reference/solidworks/ under clean snake_case names
(every part, custom and purchased - the immutable frame/size reference), seed vendor/ for
purchased parts, and write reference/manifest.json. The cycloidal drive's parts are not
SolidWorks exports: tools/cycloidal/import_cadquery.py owns them (their entries are kept).

    ./cadtool python tools/reference/import_solidworks.py --src <the SolidWorks export tree> [--force]

Copies go through the explicit map in lib/reference.py - never shell globs: the source
names contain spaces, parentheses, a trailing space and a Cyrillic configuration name.
reference/*.step are immutable inputs; vendor/*.step may be replaced by better models (see
vendor/CLAUDE.md); parts/*.step are regenerated from Python. Re-run (without --force) after
tools/reference/extract_placements.py and after replacing a vendor file, so the manifest is current.
"""
from __future__ import annotations

import argparse
import pathlib
import shutil
import sys

from lib import manifest as M
from lib import reference as R


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=pathlib.Path, required=True,
                    help="the SolidWorks export tree (the directory holding the paths lib/reference.py names)")
    ap.add_argument("--force", action="store_true", help="overwrite existing copies")
    args = ap.parse_args(argv)
    args.src = args.src.expanduser()
    if not args.src.is_dir():
        print(f"source dir not found: {args.src}", file=sys.stderr)
        return 1

    R.REF_SOLIDWORKS_DIR.mkdir(parents=True, exist_ok=True)
    R.VENDOR_DIR.mkdir(exist_ok=True)
    jobs = [(name, prod, rel, "custom") for name, (prod, rel) in R.CUSTOM.items()]
    others = R.CYCLOIDAL_PARTS | R.NATIVE_PARTS
    jobs += [(name, prod, rel, "cots") for name, (prod, rel) in R.COTS.items() if name not in others]

    # The cycloidal drive's entries are owned by tools/cycloidal/import_cadquery.py, the native parts' by
    # tools/reference/import_native.py - keep them.
    existing = M.read()["parts"]
    manifest = {
        "monolith": None,
        "parts": {name: entry for name, entry in existing.items() if name in others},
    }
    monolith = args.src / R.MONOLITH_NAME
    if monolith.exists():
        manifest["monolith"] = {"file": R.MONOLITH_NAME, "bytes": monolith.stat().st_size, "sha256": R.sha256(monolith)}

    missing: list[str] = []
    for name, prod, rel, kind in jobs:
        ref = R.path_of(name)                     # immutable SolidWorks geometry (frame + size reference), reference/solidworks/
        vendor = R.VENDOR_DIR / f"{name}.step"    # COTS only: the current best vendor model (may be replaced)
        if rel is None:
            # Extracted from the full assembly by tools/reference/extract_placements.py into vendor/; that
            # extraction IS the SolidWorks reference, so mirror it into reference/ once.
            if not vendor.exists():
                print(f"{name:24s} {kind:7s} (not yet extracted - run tools/reference/extract_placements.py)")
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
        entry = M.entry(kind, ref, product=prod, source=rel, vendor=vendor if kind == "cots" else None)
        manifest["parts"][name] = entry
        vend = "" if kind != "cots" else ("  vendor=reference" if entry["vendor"]["same_as_reference"] else "  vendor=REPLACED")
        print(f"{name:24s} {kind:7s} {action:9s} {entry['units']:4s} solids={entry['solids']:2d} "
              f"vol={entry['solid_volume']:12.1f}  size={entry['bbox_size']}{vend}")

    M.write(manifest)
    print(f"wrote {M.MANIFEST_PATH} ({len(manifest['parts'])} entries)")
    if missing:
        print("MISSING sources:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
