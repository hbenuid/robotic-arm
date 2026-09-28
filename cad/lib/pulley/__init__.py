"""The printed GT2 pulleys: the 90T pulley (gt2_pulley_90t) as parametric build123d, and the GT2 groove and toothed ring
a printed pulley carries.

Pure-Python config here (lib/pulley/params.py); the build123d builders in lib/pulley/teeth.py and body.py (imported
explicitly by their users, so importing this package never pulls in OCCT). Never imports lib/params.py.
"""
from lib.pulley.params import DEFAULT, LEGACY, PulleyParams  # noqa: F401
