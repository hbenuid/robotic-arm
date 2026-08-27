"""Extract every occurrence placement from the SolidWorks full-assembly STEP into
reference/placements.json, and flatten the NEMA 17 pancake sub-assembly into ONE vendor
part (vendor/nema17_pancake.step).

    ./cadtool python tools/extract_placements.py \
        [--monolith "~/Documents/arm_assembly_organized/final Arm Assembly Fully Movable.STEP"] \
        [--out reference/placements.json] [--pancake-out vendor/nema17_pancake.step]

build123d.import_step() walks the STEP's XCAF document and keeps the hierarchy: each node
is a Compound whose .label is the (mangled) product name and whose .location is the
placement RELATIVE to its parent; world = parent_world * rel. The full assembly is an
inch-unit file; OCCT converts it to mm on import. Skipped nodes (the cycloidal drive) are
recorded with their world pose so they can be re-attached later.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import build123d  # noqa: E402
from build123d import BoundBox, Compound, Location, export_step, import_step  # noqa: E402
from lib import reference as R  # noqa: E402

ROOT_LABEL = R.clean_label("final Arm Assembly Fully Movable")
COLLAPSED = {"nema17_pancake"}   # sub-assemblies flattened into ONE COTS part


def loc_json(loc: Location) -> dict:
    pos, rot = (tuple(v) for v in tuple(loc))   # (position Vector, XYZ-Euler-degrees Vector)
    t = loc.wrapped.Transformation()
    return {
        "position": [round(v, 6) for v in pos],
        "rotation_xyz_deg": [round(v, 6) for v in rot],
        "matrix_3x4": [[round(t.Value(i, j), 9) for j in (1, 2, 3, 4)] for i in (1, 2, 3)],
    }


def world_bbox(node, parent_world: Location):
    """Bounding box of `node` in the world frame (node.wrapped carries only its relative
    location; TopoDS_Shape.Moved composes the parent's world location without copying)."""
    bb = BoundBox.from_topo_ds(node.wrapped.Moved(parent_world.wrapped))
    return ([round(v, 3) for v in (bb.min.X, bb.min.Y, bb.min.Z)],
            [round(v, 3) for v in (bb.size.X, bb.size.Y, bb.size.Z)])


def count_leaves(node) -> int:
    return 1 if not node.children else sum(count_leaves(c) for c in node.children)


class Extractor:
    def __init__(self, pancake_out: pathlib.Path | None):
        self.records: list[dict] = []
        self.skipped: list[dict] = []
        self.ordinal: dict[str, int] = {}
        self.pancake_out = pancake_out

    def key_for(self, name: str) -> str:
        self.ordinal[name] = self.ordinal.get(name, 0) + 1
        return f"{name}#{self.ordinal[name]}"

    def record(self, key, part, kind, path_s, parent_key, node, parent_world, world) -> None:
        bmin, bsize = world_bbox(node, parent_world)
        self.records.append({
            "key": key, "part": part, "kind": kind, "path": path_s, "parent": parent_key,
            "label_in_monolith": node.label,
            "solids": len(node.solids()), "solid_volume": round(R.solid_volume(node), 3),
            "rel": loc_json(node.location), "world": loc_json(world),
            "world_bbox_min": bmin, "world_bbox_size": bsize,
        })

    def walk(self, node, parent_world: Location, parent_key: str | None, path: tuple[int, ...]) -> None:
        label = node.label
        world = parent_world * node.location
        path_s = ".".join(map(str, path))
        if label in R.SKIPPED_LABELS:
            bmin, bsize = world_bbox(node, parent_world)
            self.skipped.append({
                "path": path_s, "label": label, "reason": R.SKIPPED_LABELS[label],
                "leaves": count_leaves(node), "solids": len(node.solids()),
                "solid_volume": round(R.solid_volume(node), 3),
                "rel": loc_json(node.location), "world": loc_json(world),
                "world_bbox_min": bmin, "world_bbox_size": bsize,
            })
            return
        if label in R.LABEL_TO_MODULE:
            module = R.LABEL_TO_MODULE[label]
            key = self.key_for(module)
            self.record(key, module, "module", path_s, parent_key, node, parent_world, world)
            for i, child in enumerate(node.children, 1):
                self.walk(child, world, key, path + (i,))
            return
        part = R.LABEL_TO_PART.get(label)
        if part is None:
            raise SystemExit(f"unmapped product label {label!r} at {path_s} - add it to lib/reference.py")
        key = self.key_for(part)
        self.record(key, part, "part", path_s, parent_key, node, parent_world, world)
        if part in COLLAPSED and self.pancake_out is not None and self.ordinal[part] == 1:
            # node.solids() are in the PARENT frame (rel * local); undo rel to get the
            # sub-assembly's own frame, the frame placements.json places it in.
            flat = Compound([s.moved(node.location.inverse()) for s in node.solids()])
            flat.label = part
            self.pancake_out.parent.mkdir(exist_ok=True)
            export_step(flat, str(self.pancake_out))
            print(f"wrote {self.pancake_out}: {len(flat.solids())} solids")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--monolith", type=pathlib.Path, default=R.DEFAULT_SOURCE_DIR / R.MONOLITH_NAME)
    ap.add_argument("--out", type=pathlib.Path, default=R.REF_DIR / "placements.json")
    ap.add_argument("--pancake-out", type=pathlib.Path, default=R.VENDOR_DIR / "nema17_pancake.step")
    ap.add_argument("--no-pancake", action="store_true", help="do not (re)write the pancake vendor STEP")
    args = ap.parse_args(argv)
    monolith = args.monolith.expanduser()
    if not monolith.exists():
        print(f"full-assembly STEP not found: {monolith}", file=sys.stderr)
        return 1

    print(f"importing {monolith.name} ({monolith.stat().st_size / 1e6:.1f} MB) ...")
    root = import_step(str(monolith))
    if root.label != ROOT_LABEL:
        print(f"warning: root label {root.label!r} != {ROOT_LABEL!r}")

    ex = Extractor(None if args.no_pancake else args.pancake_out)
    for i, child in enumerate(root.children, 1):
        ex.walk(child, root.location, None, (1, i))

    parts = [o for o in ex.records if o["kind"] == "part"]
    expected = {
        "leaf_occurrences": len(parts),
        "solids": sum(o["solids"] for o in parts),
        "solid_volume": round(sum(o["solid_volume"] for o in parts), 3),
    }
    out = {
        "source": {
            "file": monolith.name, "bytes": monolith.stat().st_size,
            "sha256": hashlib.sha256(monolith.read_bytes()).hexdigest(),
            "units": "inch (OCCT converts to mm on import)",
            "extracted": datetime.date.today().isoformat(),
            "build123d": build123d.__version__,
        },
        "rotation_convention": "build123d Location.to_tuple(): intrinsic XYZ Euler angles, degrees; "
                               "rebuild with Location(position, rotation_xyz_deg). matrix_3x4 = rows of the "
                               "gp_Trsf (rotation | translation) for other consumers.",
        "frames": "rel = relative to the parent node (what assemblies/*.py compose); world = arm frame.",
        "root_label": root.label,
        "expected": expected,
        "occurrences": ex.records,
        "skipped": ex.skipped,
    }
    args.out.parent.mkdir(exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"wrote {args.out}: {len(ex.records)} records "
          f"({expected['leaf_occurrences']} parts, {len(ex.records) - expected['leaf_occurrences']} modules), "
          f"{len(ex.skipped)} skipped; solids={expected['solids']} volume={expected['solid_volume']}")
    for o in ex.records:
        p = o["world"]["position"]
        print(f"  {o['key']:26s} {o['kind']:6s} path={o['path']:8s} parent={str(o['parent']):10s} "
              f"solids={o['solids']:2d} world=({p[0]:8.2f},{p[1]:8.2f},{p[2]:8.2f})")
    for s in ex.skipped:
        p = s["world"]["position"]
        print(f"  SKIPPED {s['label']} path={s['path']} leaves={s['leaves']} solids={s['solids']} "
              f"world=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) - {s['reason']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
