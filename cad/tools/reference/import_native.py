"""Accept the current build of a NATIVE part as its reference: write reference/native/<name>.step and its
reference/manifest.json entry.

    ./cadtool python tools/reference/import_native.py [--only NAME ...] [--force]

A native part (lib/reference.py NATIVE - designed here in build123d, no SolidWorks or CadQuery origin) and a
native purchased part (NATIVE_COTS - no vendor model, the envelope IS the geometry) have no external
reference to match, so their reference is the build the author ACCEPTED: this tool builds the part
in-process (parts.build(name); the COTS envelope via the part's _envelope()) and exports it. Without --force
an existing reference is kept (the manifest entry is refreshed from the file), so a later change of the
design fails tests/test_reference_match.py until the new geometry is accepted with --force - the lock the
SolidWorks parts get from their exports.

Owns exactly the manifest entries of NATIVE | NATIVE_COTS (kind "native" / "cots", origin "native");
tools/reference/import_solidworks.py and tools/cycloidal/import_cadquery.py keep them. build123d's STEP
writer stamps the time into the header, so a reference is written ONCE, on one machine, and committed as an
LFS object (root .gitignore re-admits reference/**/*.step) - never regenerated on the other machine.
"""
from __future__ import annotations

import argparse

from build123d import export_step

import parts
from lib import manifest as M
from lib import reference as R


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", default=None, help="subset of names")
    ap.add_argument("--force", action="store_true", help="overwrite an existing reference (accept the new geometry)")
    args = ap.parse_args(argv)
    jobs = [(n, "native", b) for n, b in R.NATIVE.items()] + [(n, "cots", b) for n, b in R.NATIVE_COTS.items()]
    if not jobs:
        print("lib/reference.py NATIVE / NATIVE_COTS are empty - nothing to accept")
        return 0
    R.REF_NATIVE_DIR.mkdir(parents=True, exist_ok=True)
    manifest = M.read()
    for name, kind, builder in jobs:
        if args.only and name not in args.only:
            continue
        ref = R.path_of(name)
        if ref.exists() and not args.force:
            action = "kept"
        else:
            mod = parts.load(name)
            shape = mod._envelope() if kind == "cots" else parts.build(name)
            shape.label = name
            export_step(shape, str(ref))
            action = "accepted"
        vendor = R.VENDOR_DIR / f"{name}.step"      # a native COTS part has none - the envelope is the geometry
        entry = M.entry(kind, ref, product=parts.load(name).__doc__.splitlines()[0].strip(), source=builder,
                        vendor=vendor if kind == "cots" and vendor.exists() else None, origin="native")
        manifest["parts"][name] = entry
        print(f"{name:24s} {kind:7s} {action:9s} solids={entry['solids']:2d} vol={entry['solid_volume']:12.1f}  size={entry['bbox_size']}")
    M.write(manifest)
    print(f"wrote {M.MANIFEST_PATH} ({len(manifest['parts'])} entries)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
