"""Copy the SolidWorks per-part STEP exports into reference/ (custom parts) and vendor/
(purchased parts) under clean snake_case names, and write reference/manifest.json.

    ./cadtool python tools/import_reference.py [--src DIR] [--force]

Copies go through the explicit map in lib/reference.py - never shell globs: the source
names contain spaces, parentheses, a trailing space and a Cyrillic configuration name.
reference/*.step and vendor/*.step are immutable inputs; parts/*.step are regenerated from
Python. Re-run (without --force) after tools/extract_placements.py so the manifest also
describes the extracted vendor/nema17_pancake.step.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from build123d import import_step  # noqa: E402
from lib import reference as R  # noqa: E402

MANIFEST_PATH = R.REF_DIR / "manifest.json"


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
    shape = import_step(str(path))
    bb = shape.bounding_box()
    return {
        "solids": len(shape.solids()),
        "solid_volume": round(R.solid_volume(shape), 3),
        "bbox_min": [round(v, 3) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
        "bbox_size": [round(v, 3) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
    }


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
    jobs = [(name, prod, rel, R.REF_DIR / f"{name}.step", "reference") for name, (prod, rel) in R.CUSTOM.items()]
    jobs += [(name, prod, rel, R.VENDOR_DIR / f"{name}.step", "vendor") for name, (prod, rel) in R.COTS.items()]

    manifest = {
        "source_dir": str(args.src),
        "monolith": None,
        "parts": {},
    }
    monolith = args.src / R.MONOLITH_NAME
    if monolith.exists():
        manifest["monolith"] = {"file": R.MONOLITH_NAME, "bytes": monolith.stat().st_size, "sha256": sha256(monolith)}

    missing: list[str] = []
    for name, prod, rel, dst, kind in jobs:
        if rel is None:  # extracted from the full assembly by tools/extract_placements.py
            if not dst.exists():
                print(f"{name:24s} {kind:9s} (not yet extracted - run tools/extract_placements.py)")
                continue
            action = "extracted"
        else:
            src = args.src / rel
            if not src.exists():
                missing.append(f"{name}: {src}")
                continue
            if dst.exists() and not args.force:
                action = "kept"
            else:
                shutil.copyfile(src, dst)
                action = "copied"
        entry = {
            "kind": kind,
            "product": prod,
            "source": rel,
            "bytes": dst.stat().st_size,
            "sha256": sha256(dst),
            "units": step_units(dst),
            **describe(dst),
        }
        manifest["parts"][name] = entry
        print(f"{name:24s} {kind:9s} {action:9s} {entry['units']:4s} solids={entry['solids']:2d} "
              f"vol={entry['solid_volume']:12.1f}  size={entry['bbox_size']}")

    MANIFEST_PATH.write_text(json.dumps(manifest, indent=1, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {MANIFEST_PATH} ({len(manifest['parts'])} entries)")
    if missing:
        print("MISSING sources:\n  " + "\n  ".join(missing), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
