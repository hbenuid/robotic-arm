"""./cadtool inspect - geometry facts of a saved STEP, and a geometric diff of two of them.

    ./cadtool inspect <file.step> [--planes] [--json]
    ./cadtool inspect diff <a.step> <b.step> [--tol 1e-6]

A LOCAL tool: cadgen 0.6.5 removed `cadgen step inspect` (and says there is no replacement command) -
checks are Python scripts over `cadgen.read_scene` + `cadgen.geometry`. This one covers the two jobs
this repo used the old verb for: reading a reference / vendor STEP before converting or swapping it
(leaf refs, solids, faces, volume, bbox; `--planes` lists the planar faces to find its frame from) and
proving that a regenerated STEP is the same geometry (`diff`, exit 1 when it is not). The refs printed
(`#o1.2`, `#o1.2.f7`) are the ones the CAD Viewer shows. Distances, clearances and overlaps are not
here: `cadgen.geometry.closest_points` / `overlap_volume` in a test.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

from cadgen import read_scene

from lib.reference import solid_volume

# First arguments of the removed `cadgen step inspect`, kept only to teach the new syntax.
RETIRED_VERBS = ("refs", "measure", "align", "frame", "interfere", "validate")


def _xyz(v) -> list[float]:
    return [v.X, v.Y, v.Z]


def facts(path: str | pathlib.Path) -> dict:
    """One row per leaf occurrence (document order, world coordinates) plus the document totals."""
    scene = read_scene(str(path))
    rows, lo, hi = [], None, None
    for leaf in scene.leaves():
        shape = leaf.shape()
        bb = shape.bounding_box()
        lo = _xyz(bb.min) if lo is None else [min(a, b) for a, b in zip(lo, _xyz(bb.min))]
        hi = _xyz(bb.max) if hi is None else [max(a, b) for a, b in zip(hi, _xyz(bb.max))]
        rows.append({
            "ref": leaf.ref,
            "label": leaf.label,
            "prototype_id": leaf.prototype_id,
            "solids": len(shape.solids()),
            "faces": len(shape.faces()),
            "volume": solid_volume(shape),
            "bbox_size": _xyz(bb.size),
            "bbox_center": _xyz(bb.center()),
        })
    return {
        "file": str(path),
        "document_hash": scene.document_hash,
        "leaves": rows,
        "totals": {
            "leaves": len(rows),
            "solids": sum(r["solids"] for r in rows),
            "faces": sum(r["faces"] for r in rows),
            "volume": sum(r["volume"] for r in rows),
            "bbox_min": lo or [0.0, 0.0, 0.0],
            "bbox_size": [h - l for l, h in zip(lo, hi)] if rows else [0.0, 0.0, 0.0],
        },
    }


def planes(path: str | pathlib.Path) -> list[dict]:
    """Every planar face as `normal . x = offset` (world coordinates) - what a part's frame is read from."""
    from build123d import GeomType

    scene = read_scene(str(path))
    found = []
    for leaf in scene.leaves():
        for entity in leaf.entities("face"):
            face = entity.shape()
            if face.geom_type != GeomType.PLANE:
                continue
            normal, center = face.normal_at(), face.center()
            found.append({"ref": entity.ref, "normal": _xyz(normal), "offset": normal.dot(center), "area": face.area})
    return found


def _close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


def diff(a: str | pathlib.Path, b: str | pathlib.Path, tol: float = 1e-6) -> list[str]:
    """Differences between two STEPs leaf by leaf (document order); empty = the same geometry within
    `tol` (relative above 1, absolute below - mm, mm3). Labels and refs are compared exactly."""
    fa, fb = facts(a), facts(b)
    out = []
    if fa["totals"]["leaves"] != fb["totals"]["leaves"]:
        out.append(f"leaves: {fa['totals']['leaves']} != {fb['totals']['leaves']}")
    for ra, rb in zip(fa["leaves"], fb["leaves"]):
        where = f"{ra['ref']} {ra['label']}"
        for key in ("ref", "label", "solids", "faces"):
            if ra[key] != rb[key]:
                out.append(f"{where}: {key} {ra[key]!r} != {rb[key]!r}")
        if not _close(ra["volume"], rb["volume"], tol):
            out.append(f"{where}: volume {ra['volume']!r} != {rb['volume']!r}")
        for key in ("bbox_size", "bbox_center"):
            if not all(_close(x, y, tol) for x, y in zip(ra[key], rb[key])):
                out.append(f"{where}: {key} {ra[key]} != {rb[key]}")
    return out


def _fmt(values) -> str:
    return "(" + ", ".join(f"{round(v, 3) + 0.0:.3f}" for v in values) + ")"   # + 0.0: no "-0.000"


def _print_facts(report: dict) -> None:
    t = report["totals"]
    print(f"{report['file']}  {report['document_hash'][:12]}  leaves {t['leaves']}  solids {t['solids']}  "
          f"faces {t['faces']}  volume {t['volume']:.3f}")
    print(f"  bbox min {_fmt(t['bbox_min'])}  size {_fmt(t['bbox_size'])}")
    for r in report["leaves"]:
        print(f"  {r['ref']}  {r['label']}  {r['prototype_id']}  solids {r['solids']}  faces {r['faces']}  "
              f"volume {r['volume']:.3f}  size {_fmt(r['bbox_size'])}  centre {_fmt(r['bbox_center'])}")


def main(argv: list[str]) -> int:
    if argv and argv[0] in RETIRED_VERBS:
        print(f"cadtool inspect: `{argv[0]}` went with cadgen 0.6.5's `cadgen step inspect`. Now:\n"
              "  ./cadtool inspect <file.step> [--planes] [--json]\n"
              "  ./cadtool inspect diff <a.step> <b.step> [--tol 1e-6]\n"
              "Distances / overlaps: cadgen.geometry.closest_points / overlap_volume in a test.", file=sys.stderr)
        return 2
    if argv and argv[0] == "diff":
        parser = argparse.ArgumentParser(prog="cadtool inspect diff")
        parser.add_argument("a")
        parser.add_argument("b")
        parser.add_argument("--tol", type=float, default=1e-6)
        args = parser.parse_args(argv[1:])
        differences = diff(args.a, args.b, args.tol)
        for line in differences:
            print(line)
        print(f"{args.a} vs {args.b}: " + (f"{len(differences)} difference(s)" if differences else f"same geometry (tol {args.tol:g})"))
        return 1 if differences else 0
    parser = argparse.ArgumentParser(prog="cadtool inspect", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file")
    parser.add_argument("--planes", action="store_true", help="also list the planar faces (normal, offset, area)")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    report = facts(args.file)
    if args.planes:
        report["planes"] = planes(args.file)
    if args.json:
        print(json.dumps(report, indent=1))
        return 0
    _print_facts(report)
    for p in report.get("planes", ()):
        print(f"  {p['ref']}  normal {_fmt(p['normal'])}  offset {p['offset']:.3f}  area {p['area']:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
