"""Geometry helpers shared by the geometry tests (tests/cycloidal/, tests/forearm/, tests/upper_arm/, tests/base/,
tests/test_mounts.py, tests/test_sweeps.py), and the standalone modules' colour check (tests/test_assembly.py, the drives' own tests). The
drive's own config helpers stay in tests/cycloidal/helpers.py. A sweep - the same parts measured at many poses (the
joint sweeps, the home sensors' travel) - measures with `Reach` / `distance_under` / `nearest_under`: the exact
distance on the faces in reach of each other, where a whole-solid query spends most of its time on faces far apart.

CadQuery -> build123d idioms from the drive's port (see docs/cycloidal_drive.md "Port notes"):
  .val().isInside(v, tol)       -> is_inside(solid, x, y, z)
  a.intersect(b).val().Volume() -> interference(a, b)
  .val().Volume()               -> lib.reference.solid_volume (never Compound.volume)
  .faces("<Z"/">Z").val()       -> end_face(shape, "min"/"max")
  .section(height=z).Area()     -> section_area(solid, z)
"""
from __future__ import annotations

from build123d import Box, Color, Compound, Cylinder, GeomType, Location, Pos, PositionMode, Shape, Vertex

import parts
from assemblies._occurrences import colour
from lib import reference as R
from lib.geom import align_min


def is_inside(solid: Shape, x: float, y: float, z: float, tol: float = 1e-6) -> bool:
    return solid.is_inside((x, y, z), tol)   # build123d has is_inside on Solid / Compound, not Shape  # ty: ignore[missing-attribute]


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
    from OCP.collections import List_TopoDS_Shape  # OCCT 8: TopTools_ListOfShape

    box_a, box_b = Bnd_Box(), Bnd_Box()
    BRepBndLib.Add_s(a.wrapped, box_a, False)   # from the geometry: the shapes' meshes stay as they are
    BRepBndLib.Add_s(b.wrapped, box_b, False)
    if box_a.IsOut(box_b):
        return 0.0
    args, tools = List_TopoDS_Shape(), List_TopoDS_Shape()
    args.Append(a.wrapped)
    tools.Append(b.wrapped)
    op = BRepAlgoAPI_Common()
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetNonDestructive(True)
    op.SetFuzzyValue(COINCIDENT_MM)
    op.Build()
    return R.solid_volume(Compound(op.Shape())) if op.IsDone() else 0.0


JITTER = ((1.0, 0.7, 1.3), (-1.1, 0.9, -0.6), (0.4, -1.2, 0.8))   # x 0.01 mm: distance(), a re-checked boolean


def distance(a: Shape, b: Shape) -> float:
    """a.distance_to(b), its false zeros caught: BRepExtrema now and then reads 0 at one exact pose of two solids that
    neither touch nor overlap (its inside-the-solid check misfires - forearm_roll -140 / -145 and elbow_pitch +92, 1.0 /
    1.2 mm either side; the wrist 90T against the wrist home clip at wrist_pitch +10, 2.4). A zero is measured again
    with the pose moved 0.01 mm three ways (JITTER) and the median kept: a false zero belongs to one pose, a real touch
    stays within the jitter."""
    d = a.distance_to(b)
    if d >= 0.05:
        return d
    return sorted(a.moved(Location(tuple(0.01 * v for v in j))).distance_to(b) for j in JITTER)[1]


_CORNERS = ((0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1))


def _explore(shape, kind):
    from OCP.TopExp import TopExp_Explorer

    ex = TopExp_Explorer(shape, kind)
    while ex.More():
        yield ex.Current()
        ex.Next()


def _box(shape) -> tuple:
    """(low, high) corners of the shape's enclosing box (BRepBndLib.Add from the geometry: never too small)."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, False)
    p, q = b.CornerMin(), b.CornerMax()
    return (p.X(), p.Y(), p.Z()), (q.X(), q.Y(), q.Z())


class Reach:
    """A solid, or a compound of solids, to measure against others at many poses (a sweep) with `distance_under`: its
    faces, their enclosing boxes and, for a flat one, its plane (unit normal, offset: n.x = offset; NaN for a curved
    one), its solids with their boxes and a vertex of each - all in the shape's own frame (its geometry without its
    location), gathered once; `location` is where the shape stood."""

    def __init__(self, shape: Shape):
        import numpy as np
        from OCP.BRep import BRep_Tool
        from OCP.BRepAdaptor import BRepAdaptor_Surface
        from OCP.GeomAbs import GeomAbs_Plane
        from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_VERTEX
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS

        self.location = shape.location
        self.base = shape.wrapped.Located(TopLoc_Location())
        self.faces = [TopoDS.Face(f) for f in _explore(self.base, TopAbs_FACE)]
        boxes = [_box(f) for f in self.faces]
        self.lo = np.array([lo for lo, _ in boxes]).reshape(-1, 3)
        self.hi = np.array([hi for _, hi in boxes]).reshape(-1, 3)
        self.corners = self.lo[:, None, :] + (self.hi - self.lo)[:, None, :] * np.array(_CORNERS, dtype=float)[None]
        self.normal = np.full((len(self.faces), 3), np.nan)
        self.offset = np.full(len(self.faces), np.nan)
        for i, f in enumerate(self.faces):
            surface = BRepAdaptor_Surface(f)     # carries the face's own location
            if surface.GetType() == GeomAbs_Plane:
                plane = surface.Plane()
                n, p = plane.Axis().Direction(), plane.Location()
                self.normal[i] = (n.X(), n.Y(), n.Z())
                self.offset[i] = n.X() * p.X() + n.Y() * p.Y() + n.Z() * p.Z()
        # the whole shape's box (round its faces' boxes) by its 8 corners
        lo, hi = self.lo.min(axis=0, initial=np.inf), self.hi.max(axis=0, initial=-np.inf)
        self.box_corners = lo + (hi - lo) * np.array(_CORNERS, dtype=float)
        self.solids = []      # (solid, its box's low and high corners, one of its vertices, its classifier: loaded once)
        for s in _explore(self.base, TopAbs_SOLID):
            lo, hi = _box(s)
            self.solids.append([s, lo, hi, BRep_Tool.Pnt_s(TopoDS.Vertex(next(_explore(s, TopAbs_VERTEX)))), None])

    def at(self, move: Location | None = None) -> Location:
        """Its location moved by `move` (a move in its parent's frame, as Shape.moved() takes it)."""
        return self.location if move is None else move * self.location

    def shape_at(self, at: Location) -> Shape:
        return Compound.cast(self.base.Located(at.wrapped))


def _inside(point, trsf, solids) -> bool:
    """`point` (moved by trsf) inside one of `solids` (Reach.solids) - a point outside a solid's box is outside it; a
    solid's classifier is loaded on its first use and kept (loading it is most of what a classification costs)."""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_IN

    p = point.Transformed(trsf)
    xyz = (p.X(), p.Y(), p.Z())
    for entry in solids:
        solid, lo, hi = entry[:3]
        if all(lo[i] <= xyz[i] <= hi[i] for i in range(3)):
            if entry[4] is None:
                entry[4] = BRepClass3d_SolidClassifier(solid)
            entry[4].Perform(p, 1e-7)
            if entry[4].State() == TopAbs_IN:
                return True
    return False


def _in_reach(lo, hi, olo, ohi, limit: float):
    """Which of the boxes (lo, hi rows) come nearer than `limit` to the box (olo, ohi)."""
    import numpy as np

    d = np.maximum(0.0, np.maximum(lo - ohi, olo - hi))
    return (d * d).sum(axis=-1) < limit * limit


def _off_plane(corners, normal, offset):
    """How far boxes (their 8 corners, (n, 8, 3)) stand off planes (unit normals (m, 3), offsets (m,)): the least
    corner's distance where all 8 stand on one side, 0 where the plane cuts the box or the face is curved (a NaN
    plane) - (n, m). A flat face lies in its plane, so it is at least that far from anything in the box."""
    import numpy as np

    s = np.einsum("ikc,jc->ijk", corners, np.nan_to_num(normal)) - np.nan_to_num(offset)[None, :, None]
    lb = np.where(s.min(axis=2) > 0.0, s.min(axis=2), np.where(s.max(axis=2) < 0.0, -s.max(axis=2), 0.0))
    return np.where(np.isnan(offset)[None, :], 0.0, lb)


def distance_under(a: Reach, b: Reach, limit: float, at_a: Location | None = None, at_b: Location | None = None) -> float:
    """The distance between a at at_a and b at at_b (their locations, by default where they stood: Reach.at()) - exact
    where it is under `limit`, else `limit`. What closest_points / distance() measure, on fewer faces
    (`_face_distance`); a reading under 0.05 is measured again by distance() on the shapes themselves - the solids,
    its false zeros caught."""
    at_a, at_b = a.at() if at_a is None else at_a, b.at() if at_b is None else at_b
    d = _face_distance(a, b, limit, at_a, at_b)
    return distance(a.shape_at(at_a), b.shape_at(at_b)) if d < 0.05 else d


def _face_distance(a: Reach, b: Reach, limit: float, at_a: Location, at_b: Location) -> float:
    """The distance between a at at_a and b at at_b, exact where under `limit`, else `limit`, measured on their faces:
    a's face boxes go into b's frame (their corners moved - the moved box still holds the face), and a pair of faces
    cannot be nearer than the gap between their boxes, nor than either box stands off the other face's plane where that
    face is flat (`_off_plane`); BRepExtrema measures the faces of the pairs left under `limit`, both sides. The solids'
    insides count as closest_points has them: a solid inside the other (no face in reach, or none touching) is at 0."""
    import numpy as np
    from OCP.BRep import BRep_Builder
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS_Compound

    rel = at_b.wrapped.Transformation().Inverted().Multiplied(at_a.wrapped.Transformation())   # a's frame -> b's
    r = np.array([[rel.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    t = np.array([rel.Value(i, 4) for i in (1, 2, 3)])
    whole = a.box_corners @ r.T + t                              # the whole shapes' boxes first: neither inside the
    if not _in_reach(whole.min(axis=0), whole.max(axis=0), b.box_corners[0], b.box_corners[-1], limit):
        return limit                                             # other, no face of one in reach of the other
    moved = a.corners @ r.T + t                                  # a's face boxes in b's frame, (faces, 8, 3)
    alo, ahi = moved.min(axis=1), moved.max(axis=1)              # and the world-axis boxes round them
    d = limit
    if len(alo) and len(b.lo):
        # a's faces in reach of b's box, b's in reach of a's; then the pairs among those
        ka = np.nonzero(_in_reach(alo, ahi, b.lo.min(axis=0), b.hi.max(axis=0), limit))[0]
        kb = np.nonzero(_in_reach(b.lo, b.hi, alo.min(axis=0), ahi.max(axis=0), limit))[0]
        pair = np.zeros((len(ka), len(kb)), dtype=bool)
        step = max(1, 50_000 // max(1, len(kb)))                  # rows at a time: (rows, kb, 8) stays small
        for s in range(0, len(ka), step):
            rows = ka[s:s + step]
            near = _in_reach(alo[rows][:, None, :], ahi[rows][:, None, :], b.lo[kb][None], b.hi[kb][None], limit)
            if near.any():
                # b's flat faces against a's boxes, a's flat faces (their planes in b's frame) against b's boxes
                na = a.normal[rows] @ r.T
                off = _off_plane(moved[rows], b.normal[kb], b.offset[kb])
                near &= np.maximum(off, _off_plane(b.corners[kb], na, a.offset[rows] + na @ t).T) < limit
            pair[s:s + step] = near
        ia, ib = ka[pair.any(axis=1)], kb[pair.any(axis=0)]
        if len(ia):
            # one query over both sides' faces in pairs: BRepExtrema's own pruning across them beats a query per face
            build = BRep_Builder()
            fa, fb = TopoDS_Compound(), TopoDS_Compound()
            build.MakeCompound(fa)
            build.MakeCompound(fb)
            move = TopLoc_Location(rel)
            for i in ia:
                build.Add(fa, a.faces[i].Moved(move))
            for j in ib:
                build.Add(fb, b.faces[j])
            d = min(d, BRepExtrema_DistShapeShape(fa, fb).Value())
    if d > 0.0 and (any(_inside(s[3], rel, b.solids) for s in a.solids)
                    or any(_inside(s[3], rel.Inverted(), a.solids) for s in b.solids)):
        d = 0.0
    return d


def nearest_under(movers, fixed, limit: float, move: Location | None = None) -> float:
    """The least distance of any of `movers` (Reach-es, each moved by `move`) to any of `fixed` (Reach-es where they
    stood): exact where under `limit`, else `limit`. Each pair is measured under the least found so far - a pair farther
    than that cannot be the least -, so a sweep after the least over its poses passes min(limit, least so far)."""
    least = limit
    for m in movers:
        at = m.at(move)
        for f in fixed:
            if least <= 0.0:
                return 0.0
            least = min(least, distance_under(m, f, least, at_a=at))
    return least


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


def part_colours(node) -> dict[bool, int]:
    """Every leaf of an assembly node (the arm, a module standing alone) in its part's colour (_occurrences.colour():
    lib/style.py); returns the bought / printed leaf counts."""
    seen = {True: 0, False: 0}
    for leaf in leaves(node):
        name = leaf.label.split(":")[0]
        assert same_color(leaf, colour(name)), f"{node.label}/{leaf.label}: {tuple(leaf.color)}, not {colour(name)}"
        seen[parts.bought(name)] += 1
    return seen
