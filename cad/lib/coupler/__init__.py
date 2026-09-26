"""The J3 coupler (j3_coupler) as parametric build123d.

Pure-Python config and layout here (lib/coupler/params.py, layout.py); the build123d builder in lib/coupler/body.py
(imported explicitly by the part, so importing this package never pulls in OCCT). Never imports lib/params.py.
"""
from lib.coupler.layout import flange_bolt_points, pulley_bolt_points  # noqa: F401
from lib.coupler.params import DEFAULT, LEGACY, CouplerParams  # noqa: F401
