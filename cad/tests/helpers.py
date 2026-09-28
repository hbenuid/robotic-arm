"""Geometry helpers shared by the geometry tests (tests/cycloidal/, tests/forearm/, tests/upper_arm/, tests/base/, tests/coupler/,
tests/test_mounts.py), and the standalone modules' colour check (tests/test_assembly.py, the drives' own tests). The
drive's own config helpers stay in tests/cycloidal/helpers.py.

CadQuery -> build123d idioms from the drive's port (see docs/cycloidal_drive.md "Port notes"):
  .val().isInside(v, tol)       -> is_inside(solid, x, y, z)
  a.intersect(b).val().Volume() -> interference(a, b)
  .val().Volume()               -> lib.reference.solid_volume (never Compound.volume)
  .faces("<Z"/">Z").val()       -> end_face(shape, "min"/"max")
  .section(height=z).Area()     -> section_area(solid, z)
"""
from __future__ import annotations

from build123d import Box, Color, Compound, Cylinder, GeomType, Pos, PositionMode, Shape, Vertex

import parts
from assemblies._occurrences import BOUGHT_TINT
from lib import reference as R
from lib.geom import align_min


def is_inside(solid: Shape, x: float, y: float, z: float, tol: float = 1e-6) -> bool:
    return solid.is_inside((x, y, z), tol)


COINCIDENT_MM = 1e-5   # faces nearer than this are one face to interference() (the Common's fuzzy value)


def interference(a: Shape, b: Shape) -> float:
    """Volume of a ∩ b (0 when the boolean is empty) - the kernel's Common directly, because
    build123d 0.11 reworked `Shape.intersect` for composite operands (a placed module of 18
    solids against a part reported whole solids as "common"). Shapes whose bounding boxes do not meet
    share nothing: no boolean. Otherwise one Build, non-destructive: the operands stay untouched. A line-to-line fit
    (a bearing's OD in a seat of its own diameter: coincident cylinders) is the kernel's fragile case - on x86_64 Linux
    the Common of bearing_6806#3 in j1_link's Ø42.0 seat came back as the whole bearing - so faces within
    COINCIDENT_MM count as coincident."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepBndLib import BRepBndLib
    from OCP.TopTools import TopTools_ListOfShape

    box_a, box_b = Bnd_Box(), Bnd_Box()
    BRepBndLib.Add_s(a.wrapped, box_a, False)   # from the geometry: the shapes' meshes stay as they are
    BRepBndLib.Add_s(b.wrapped, box_b, False)
    if box_a.IsOut(box_b):
        return 0.0
    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(a.wrapped)
    tools.Append(b.wrapped)
    op = BRepAlgoAPI_Common()
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetNonDestructive(True)
    op.SetFuzzyValue(COINCIDENT_MM)
    op.Build()
    return R.solid_volume(Compound(op.Shape())) if op.IsDone() else 0.0


def end_face(shape: Shape, which: str):
    """The planar face with the lowest ("min") / highest ("max") centre Z."""
    planes = shape.faces().filter_by(GeomType.PLANE)

    def key(f):
        return f.center().Z

    return min(planes, key=key) if which == "min" else max(planes, key=key)


def radial_extent(shape: Shape) -> float:
    """max |x|, |y| of the bounding box - the CadQuery tests' 'max radial extent'."""
    bb = shape.bounding_box()
    return max(abs(bb.min.X), abs(bb.max.X), abs(bb.min.Y), abs(bb.max.Y))


def section_area(solid: Shape, z: float, size: float = 400.0, slab: float = 0.02) -> float:
    """Cross-section area at height z (thin-slab intersection volume / slab thickness)."""
    return interference(solid, Pos(0, 0, z) * Box(size, size, slab)) / slab


def probe_volume(solid: Shape, xy, z: float, size: float = 1.0, height: float = 0.5) -> float:
    """Material volume inside a small box standing on z at xy."""
    return interference(solid, Pos(xy[0], xy[1], z) * Box(size, size, height, align=align_min()))


def mesh_volume(shape: Shape, tolerance: float = 0.002, angular: float = 0.2):
    """(signed volume, centroid, triangle count) of the tessellation - a deterministic identity
    check between two BReps (OCCT's analytic volume is unreliable on the spline discs)."""
    verts, tris = shape.tessellate(tolerance, angular)
    v = cx = cy = cz = 0.0
    for a, b, c in tris:
        p, q, r = verts[a], verts[b], verts[c]
        t = p.dot(q.cross(r)) / 6.0
        v += t
        cx += t * (p.X + q.X + r.X) / 4.0
        cy += t * (p.Y + q.Y + r.Y) / 4.0
        cz += t * (p.Z + q.Z + r.Z) / 4.0
    return v, (cx / v, cy / v, cz / v), len(tris)


def spline_deviation(shape: Shape, ref: Shape, samples: int = 200) -> float:
    """Largest distance (mm) from `shape`'s B-spline edges to the matching edges of `ref` (paired by
    height, then length) - the lobe profile itself, independent of how a platform's mesher discretises it."""
    def splines(s: Shape):
        return sorted((e for e in s.edges() if e.geom_type == GeomType.BSPLINE),
                      key=lambda e: (round(e.center().Z, 3), round(e.length, 3)))
    mine, theirs = splines(shape), splines(ref)
    assert mine and len(mine) == len(theirs), f"{len(mine)} B-spline edges vs {len(theirs)} in the reference"
    return max(b.distance_to(Vertex(*a.position_at(i / samples, position_mode=PositionMode.PARAMETER)))
               for a, b in zip(mine, theirs, strict=True) for i in range(samples))


def fingerprint(shape: Shape) -> list[tuple[str, float]]:
    """Sorted (surface type, area) of every face."""
    return sorted((str(f.geom_type), round(f.area, 3)) for f in shape.faces())


def annulus(od: float, bore: float, width: float):
    """A plain annulus standing on z=0 - a bearing stand-in (the drive's simplified purchased-part model)."""
    return Cylinder(od / 2.0, width, align=align_min()) - Cylinder(bore / 2.0, width, align=align_min())


def leaves(node) -> list:
    """The leaves of an assembly node (the node itself when it has no children)."""
    return [node] if not node.children else [leaf for child in node.children for leaf in leaves(child)]


def same_color(shape, tint: str) -> bool:
    return all(abs(x - y) < 1e-6 for x, y in zip(tuple(shape.color), tuple(Color(tint)), strict=True))


def module_tints(module, tint: str) -> dict[bool, int]:
    """A standalone module: purchased parts BOUGHT_TINT, printed parts the module's TINT; returns the
    bought / printed leaf counts."""
    seen = {True: 0, False: 0}
    for leaf in leaves(module):
        bought = parts.bought(leaf.label.split(":")[0])
        assert same_color(leaf, BOUGHT_TINT if bought else tint), f"{module.label}/{leaf.label}: {tuple(leaf.color)}"
        seen[bought] += 1
    return seen
