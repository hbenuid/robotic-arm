"""The fit gauges on the printer's bed: pack() lays coupons (their footprints) onto square plates - small strips first
into the big rings' openings, then shelves, the deepest footprint first. Pure Python: it reads sizes, builds nothing."""
from __future__ import annotations

from dataclasses import dataclass

BED = 256.0      # [DESIGN] the bed's side (a 256 x 256 printer)
MARGIN = 5.0     # [DESIGN] kept clear round the bed's edge
GAP = 5.0        # [DESIGN] between two coupons on a shelf, and between shelves
RING_GAP = 3.0   # [DESIGN] round a strip nested in a ring's opening


@dataclass(frozen=True)
class Piece:
    """A coupon's footprint (w along X, d along Y, from its low corner); a ring's `opening` (the clear diameter inside
    it, 0: none) round `centre` in its own footprint."""
    key: str
    w: float
    d: float
    opening: float = 0.0
    centre: tuple = (0.0, 0.0)


@dataclass(frozen=True)
class Placed:
    key: str
    x: float     # the footprint's low corner on the bed
    y: float


def pack(pieces, bed: float = BED) -> list[list[Placed]]:
    """The pieces on as few plates as this greedy layout finds, each piece once, inside the margins."""
    usable = bed - 2.0 * MARGIN
    rings = sorted((p for p in pieces if p.opening > 0.0), key=lambda p: -p.opening)
    loose = sorted((p for p in pieces if p.opening <= 0.0), key=lambda p: -(p.w ** 2 + p.d ** 2))
    inside: dict[str, tuple] = {}
    for ring in rings:
        for p in loose:
            if p.key not in inside and (p.w ** 2 + p.d ** 2) ** 0.5 + 2.0 * RING_GAP <= ring.opening:
                inside[p.key] = (ring.key, ring.centre[0] - p.w / 2.0, ring.centre[1] - p.d / 2.0)
                break
    nested: dict[str, list] = {}
    for key, (host, dx, dy) in inside.items():
        nested.setdefault(host, []).append((key, dx, dy))
    plates: list[tuple[list[dict], list[Placed]]] = []

    def place(p: Piece):
        for shelves, items in plates:
            for sh in shelves:
                if p.d <= sh["d"] + 1e-6 and sh["x"] + p.w <= usable + 1e-6:
                    items.append(Placed(p.key, MARGIN + sh["x"], MARGIN + sh["y"]))
                    sh["x"] += p.w + GAP
                    return
            top = shelves[-1]["y"] + shelves[-1]["d"] + GAP
            if top + p.d <= usable + 1e-6 and p.w <= usable + 1e-6:
                shelves.append({"y": top, "d": p.d, "x": p.w + GAP})
                items.append(Placed(p.key, MARGIN, MARGIN + top))
                return
        if p.w > usable + 1e-6 or p.d > usable + 1e-6:
            raise ValueError(f"{p.key} ({p.w:.1f} x {p.d:.1f}) does not fit a {bed:.0f} bed")
        plates.append(([{"y": 0.0, "d": p.d, "x": p.w + GAP}], [Placed(p.key, MARGIN, MARGIN)]))

    for p in sorted((p for p in pieces if p.key not in inside), key=lambda p: (-p.d, -p.w)):
        place(p)
    out = []
    for _, items in plates:
        plate = list(items)
        for it in items:
            plate += [Placed(key, it.x + dx, it.y + dy) for key, dx, dy in nested.get(it.key, [])]
        out.append(plate)
    return out
