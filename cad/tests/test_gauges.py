"""The fit gauges (lib/gauges, tools/export_gauges.py): the list of fits, the sweeps, the coupons and the plates.

Fast: every gauge well-formed (a known builder and options it takes, every arm value in its sweep, the sweep's steps),
the clearance holes and press nut pocket lib/fasteners.py and the parts size have gauges, the titles in the label
font's characters, the packing on synthetic pieces, lib/gauges imported without the kernel. Slow: every coupon one valid
solid on the bed inside the margins, its round features at the sizes its labels print, the plates (every coupon once,
none overlapping, the nested ones inside their ring's opening), a plate's 3MF (millimetres, one named object a coupon, the process's locale kept)."""
import inspect
import itertools
import locale
import math
import re
import subprocess
import sys
import zipfile

import build123d as bd
import pytest

from lib.fasteners import M2_CLEAR, M3_CLEAR, M4_CLEAR, M5_CLEAR
from lib.gauges import coupons as C
from lib.gauges.fits import GAUGES, GROUPS
from lib.gauges.layout import BED, MARGIN, Piece, pack
from lib.reference import CAD_DIR

EPS = 1e-6
ROUND = {"seat_strip", "ring_seats", "ring_bosses", "peg_strip"}   # every size a cylinder's diameter


def _round(g) -> bool:
    shape = dict(g.options).get("shape", "round")
    return g.coupon in ROUND or (g.coupon in ("hole_strip", "side_strip") and shape in ("round", "d"))


# --- fast -------------------------------------------------------------------------------------------------------

def test_gauges_are_well_formed():
    keys = [g.key for g in GAUGES]
    assert len(keys) == len(set(keys))
    assert {g.group for g in GAUGES} == {k for k, _ in GROUPS}
    for g in GAUGES:
        params = inspect.signature(C.BUILDERS[g.coupon]).parameters
        assert set(dict(g.options)) <= set(params), (g.key, set(dict(g.options)) - set(params))
        assert g.uses and all(u.value > 0.0 and u.where for u in g.uses), g.key


@pytest.mark.parametrize("gauge", GAUGES, ids=lambda g: g.key)
def test_every_arm_value_is_in_its_sweep(gauge):
    sizes, values = gauge.sizes, gauge.values
    assert all(b - a > EPS for a, b in itertools.pairwise(sizes))
    assert all(any(abs(s - v) < EPS for s in sizes) for v in values)
    for v in values:   # every step of every arm value's own sweep
        assert all(any(abs(s - (v + k * gauge.step)) < EPS for s in sizes) for k in range(-gauge.reach, gauge.reach + 1))
    reach = gauge.reach * gauge.step
    assert min(values) - reach - EPS <= sizes[0] and sizes[-1] <= max(values) + reach + EPS

    def printed(x):
        return any(abs(s - x) < EPS for s in sizes)

    for s in sizes:   # nothing else but the one step filled between two runs
        near = min(abs(s - v) for v in values)
        assert near < reach + EPS or (printed(s - gauge.step) and printed(s + gauge.step)), (gauge.key, s)


def test_the_fasteners_have_gauges():
    """The clearance holes lib/fasteners.py defines each have a gauge, upright; the sideways M4 nut pockets' sizes are
    in the upright gauge's sweep too."""
    upright = {v for g in GAUGES if g.coupon != "side_strip" for v in g.values}
    for clear in (M2_CLEAR, M3_CLEAR, M4_CLEAR, M5_CLEAR):
        assert round(clear, 3) in upright, clear
    gauge = {g.key: g for g in GAUGES}
    assert set(gauge["m4hexs"].values) <= set(gauge["m4hex"].values)


def test_titles_use_the_label_font():
    """Upper-case ASCII, digits, a few marks and Ø: what the coupons deboss (Liberation Sans Bold has them all)."""
    for g in GAUGES:
        assert re.fullmatch(r"[A-Z0-9 ./Ø-]+", g.title), g.title
    assert C.FONT.is_file() and (C.FONT.parent / "license.txt").is_file()


def test_pack_places_every_piece_once_inside_the_bed():
    ring = Piece("ring", 96.0, 109.0, 87.0, (48.0, 61.0))
    pieces = [ring] + [Piece(f"strip{i}", 60.0 + i, 20.0 + i % 3) for i in range(14)] + [Piece("big", 230.0, 60.0)]
    plates = pack(pieces)
    placed = [p for plate in plates for p in plate]
    assert sorted(p.key for p in placed) == sorted(p.key for p in pieces)
    size = {p.key: p for p in pieces}
    for plate in plates:
        for p in plate:
            assert MARGIN - EPS <= p.x and p.x + size[p.key].w <= BED - MARGIN + EPS
            assert MARGIN - EPS <= p.y and p.y + size[p.key].d <= BED - MARGIN + EPS
    with pytest.raises(ValueError):
        pack([Piece("too_long", BED, 10.0)])


def test_gauges_import_without_the_kernel():
    probe = ("import sys, lib.gauges.fits, lib.gauges.layout; "
             "sys.exit(sorted({m.split('.')[0] for m in sys.modules if m.startswith(('build123d.', 'OCP.'))}) != [])")
    run = subprocess.run([sys.executable, "-B", "-c", probe], cwd=CAD_DIR, capture_output=True, text=True, check=False,
                         env={"PYTHONPATH": str(CAD_DIR), "PATH": ""})
    assert run.returncode == 0, (run.stdout + run.stderr).strip()


# --- slow -------------------------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def built():
    from tools import export_gauges
    coupons = export_gauges.build_all()
    return coupons, export_gauges.plates(coupons)


@pytest.mark.slow
def test_every_coupon_is_one_solid_on_the_bed(built):
    coupons, _ = built
    for c in coupons:
        assert len(c.shape.solids()) == 1 and c.shape.is_valid, c.key
        bb = c.shape.bounding_box()
        assert max(abs(bb.min.X), abs(bb.min.Y), abs(bb.min.Z)) < EPS, c.key
        assert bb.size.X <= BED - 2.0 * MARGIN and bb.size.Y <= BED - 2.0 * MARGIN, c.key


@pytest.mark.slow
def test_round_features_print_their_labels_sizes(built):
    """Each size a round gauge's coupon labels is a cylinder's diameter on it: the label says what is printed."""
    coupons, _ = built
    rounds = {g.key for g in GAUGES if _round(g)}
    checked = 0
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    for c in (c for c in coupons if c.gauge in rounds):
        diameters = [2.0 * BRepAdaptor_Surface(f.wrapped).Cylinder().Radius()
                     for f in c.shape.faces().filter_by(bd.GeomType.CYLINDER)]
        for s in c.sizes:
            assert any(abs(d - s) < 1e-4 for d in diameters), (c.key, s)
            checked += 1
    assert checked == sum(len(g.sizes) for g in GAUGES if _round(g))


@pytest.mark.slow
def test_the_plates(built):
    """Every coupon on one plate, inside the margins; two footprints meet only where a strip sits in a ring's opening."""
    coupons, plates = built
    assert sorted(c.key for plate in plates for c, _, _ in plate) == sorted(c.key for c in coupons)
    for plate in plates:
        boxes = {c.key: (x, y, x + c.size[0], y + c.size[1]) for c, x, y in plate}
        for x0, y0, x1, y1 in boxes.values():
            assert MARGIN - EPS <= min(x0, y0) and max(x1, y1) <= BED - MARGIN + EPS
        rings = [(c, x + c.centre[0], y + c.centre[1]) for c, x, y in plate if c.opening > 0.0]
        nested = {}
        for key, (x0, y0, x1, y1) in boxes.items():
            for ring, cx, cy in rings:
                far = max(math.hypot(px - cx, py - cy) for px in (x0, x1) for py in (y0, y1))
                if ring.key != key and far <= ring.opening / 2.0:
                    nested[key] = ring.key
        for a, b in itertools.combinations(boxes, 2):
            (ax0, ay0, ax1, ay1), (bx0, by0, bx1, by1) = boxes[a], boxes[b]
            if ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1:
                assert nested.get(a) == b or nested.get(b) == a, (a, b)


@pytest.mark.slow
def test_a_plate_3mf(built, tmp_path):
    from tools import export_gauges
    _, plates = built
    path = tmp_path / "plate.3mf"
    before = locale.setlocale(locale.LC_ALL)
    export_gauges.write_plate(plates[0], path, 0.05, 0.3)
    # lib3mf's write switches the process to the C locale: write_plate puts it back, or every later read without an
    # encoding in this process is ASCII (test_tooling's of the code files failed so after this test)
    assert locale.setlocale(locale.LC_ALL) == before
    with zipfile.ZipFile(path) as z:
        model = z.read("3D/3dmodel.model").decode()
    assert 'unit="millimeter"' in model
    assert sorted(re.findall(r'<object id="\d+" name="([^"]+)"', model)) == sorted(c.key for c, _, _ in plates[0])
