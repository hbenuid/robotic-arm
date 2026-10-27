"""./cadtool inspect - geometry facts of a saved STEP, and a geometric diff of two of them.

    ./cadtool inspect <file.step> [--planes] [--json]
    ./cadtool inspect diff <a.step> <b.step> [--tol 1e-6]

A LOCAL tool over `cadgen.read_step` (the STEP's tree with its instance names, each node's location in its
parent's frame) + build123d. It covers the two jobs this repo has: reading a reference / vendor STEP before
converting or swapping it (every leaf: solids, faces, volume, bbox; `--planes` lists the planar faces to find
its frame from) and proving that a regenerated STEP is the same geometry (`diff`, exit 1 when it is not). The
refs printed are cadgen 0.8's (the CAD Viewer's): a leaf's occurrence path from the root, a label its siblings
share numbered (`gripper/gripper_slider:1.2`), then `face N` (the face's ordinal in the part,
TopExp.MapShapes order) - `./cadtool cadgen inspect <file.step> "<ref>"` says what one names and measures it.
Distances, clearances and overlaps are not here: `cadgen.geometry.closest_points` / `overlap_volume` in a test.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

from lib.reference import solid_volume

# First arguments of the removed `cadgen step inspect` (cadgen 0.6.5), kept only to teach the syntax.
RETIRED_VERBS = ("refs", "measure", "align", "frame", "interfere", "validate")


def _xyz(v) -> list[float]:
    return [v.X, v.Y, v.Z]


def leaves(path: str | pathlib.Path) -> list[tuple[str, str, object]]:
    """(occurrence path, label, the leaf in world coordinates) for every leaf, in document order."""
    import build123d as bd
    from cadgen import read_step

    found = []

    def walk(node, here: str, outer):
        world = outer * node.location
        kids = list(node.children or ())
        if not kids:
            found.append((here, node.label or "", node.located(world)))
            return
        counts: dict[str, int] = {}
        for kid in kids:
            counts[kid.label] = counts.get(kid.label, 0) + 1
        seen: dict[str, int] = {}
        for kid in kids:
            n = seen[kid.label] = seen.get(kid.label, 0) + 1
            segment = f"{kid.label}.{n}" if counts[kid.label] > 1 else kid.label
            walk(kid, f"{here}/{segment}", world)

    root = read_step(str(path))
    walk(root, root.label or "", bd.Location())
    return found


def _faces(shape) -> list:
    """The part's faces by ordinal (index 0 = `face 1`), as cadgen numbers them."""
    import build123d as bd
    from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher  # OCCT 8: TopTools_IndexedMapOfShape
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopoDS import TopoDS

    mapped = IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher()
    TopExp.MapShapes_s(shape.wrapped, TopAbs_FACE, mapped)
    return [bd.Face(TopoDS.Face(mapped.FindKey(i))) for i in range(1, mapped.Extent() + 1)]


def facts(path: str | pathlib.Path) -> dict:
    """One row per leaf occurrence (document order, world coordinates) plus the document totals."""
    rows, lo, hi = [], None, None
    for ref, label, shape in leaves(path):
        bb = shape.bounding_box()
        lo = _xyz(bb.min) if lo is None else [min(a, b) for a, b in zip(lo, _xyz(bb.min), strict=True)]
        hi = _xyz(bb.max) if hi is None else [max(a, b) for a, b in zip(hi, _xyz(bb.max), strict=True)]
        rows.append({
            "ref": ref,
            "label": label,
            "solids": len(shape.solids()),
            "faces": len(_faces(shape)),
            "volume": solid_volume(shape),
            "bbox_size": _xyz(bb.size),
            "bbox_center": _xyz(bb.center()),
        })
    return {
        "file": str(path),
        "leaves": rows,
        "totals": {
            "leaves": len(rows),
            "solids": sum(r["solids"] for r in rows),
            "faces": sum(r["faces"] for r in rows),
            "volume": sum(r["volume"] for r in rows),
            "bbox_min": lo or [0.0, 0.0, 0.0],
            "bbox_size": [h - l for l, h in zip(lo, hi, strict=True)] if rows else [0.0, 0.0, 0.0],
        },
    }


def planes(path: str | pathlib.Path) -> list[dict]:
    """Every planar face as `normal . x = offset` (world coordinates) - what a part's frame is read from."""
    import build123d as bd

    found = []
    for ref, _, shape in leaves(path):
        for n, face in enumerate(_faces(shape), start=1):
            if face.geom_type != bd.GeomType.PLANE:
                continue
            normal, center = face.normal_at(), face.center()
            found.append({"ref": f"{ref}/face {n}", "normal": _xyz(normal), "offset": normal.dot(center),
                          "area": face.area})
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
    for ra, rb in zip(fa["leaves"], fb["leaves"], strict=False):   # a count mismatch is reported above; compare the common leaves
        where = ra["ref"]
        for key in ("ref", "label", "solids", "faces"):
            if ra[key] != rb[key]:
                out.append(f"{where}: {key} {ra[key]!r} != {rb[key]!r}")
        if not _close(ra["volume"], rb["volume"], tol):
            out.append(f"{where}: volume {ra['volume']!r} != {rb['volume']!r}")
        for key in ("bbox_size", "bbox_center"):
            if not all(_close(x, y, tol) for x, y in zip(ra[key], rb[key], strict=True)):
                out.append(f"{where}: {key} {ra[key]} != {rb[key]}")
    return out


def _fmt(values) -> str:
    return "(" + ", ".join(f"{round(v, 3) + 0.0:.3f}" for v in values) + ")"   # + 0.0: no "-0.000"


def _print_facts(report: dict) -> None:
    t = report["totals"]
    print(f"{report['file']}  leaves {t['leaves']}  solids {t['solids']}  faces {t['faces']}  volume {t['volume']:.3f}")
    print(f"  bbox min {_fmt(t['bbox_min'])}  size {_fmt(t['bbox_size'])}")
    for r in report["leaves"]:
        print(f"  {r['ref']}  solids {r['solids']}  faces {r['faces']}  volume {r['volume']:.3f}  "
              f"size {_fmt(r['bbox_size'])}  centre {_fmt(r['bbox_center'])}")


def main(argv: list[str]) -> int:
    if argv and argv[0] in RETIRED_VERBS:
        print(f"cadtool inspect: `{argv[0]}` went with cadgen 0.6.5's `cadgen step inspect`. Now:\n"
              "  ./cadtool inspect <file.step> [--planes] [--json]\n"
              "  ./cadtool inspect diff <a.step> <b.step> [--tol 1e-6]\n"
              "One ref: ./cadtool cadgen inspect <file.step> \"<ref>\". "
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
