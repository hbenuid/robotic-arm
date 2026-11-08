"""The fit gauges: test prints for every fit where a bought part meets a printed one - each fit printed at the arm's
own size and at steps either side of it, every size debossed beside it.

fits.py    GAUGES: every kind of fit, the arm's sizes read from the constants the parts use (Use / Gauge)
coupons.py the coupon builders (build123d, the kernel lazy), sweep(), build(gauge)
layout.py  pack(): the coupons on the bed's plates (pure Python)

tools/export_gauges.py writes them, one 3MF per plate (tools/AGENTS.md); tests/test_gauges.py checks them.
"""
