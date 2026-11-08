"""eg17_g5_gearbox, the elbow motor's 5:1 planetary (StepperOnline EG17-G5): a purchased part with no reference
(lib/reference.py NO_REFERENCE), built from its datasheet drawing (lib/motors.py GearboxParams) - THIS file locks its
numbers (measured on the build and written in) and probes the drawing's features in the part's frame (the motor's:
the output face z=0, the body -Z, the shaft +Z, the M4s on the X and Y axes, the keyway on +X). Its seat on j1_link's
web and the motor on it are tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from tests import built
from tests.helpers import is_inside

NAME = "eg17_g5_gearbox"
G = PARAMS.EG17_G5
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (1, 76741.06, (-21.0, -21.0, -56.0), (42.0, 42.0, 77.5))


def test_it_is_a_purchased_part_with_no_reference_in_the_joints_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == 1
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()"
    assert mod.MASS_G == PARAMS.EG17_G5_MASS_G == 422.0


def test_the_datasheets_numbers():
    """The drawing's dimensions (GearboxParams [DATASHEET]): a 5:1, 56 long on the NEMA 17 square, the Ø25 x 2 boss, the
    Ø8 shaft ending 21.5 from the face with its 3 x 14 key 2 short of the end and 9.2 over the shaft, 4x M4 x 8 on Ø35,
    the input end's Ø22 bore 25.5 deep and its plate 4.7 thick on the 31 square of Ø3.5; the three sections end to end;
    the keyway as deep as a 3 x 3 key less what it stands proud."""
    assert (G.ratio, G.length, G.square, G.boss_dia, G.boss_t) == (5.0, 56.0, 42.0, 25.0, 2.0)
    assert (G.shaft_dia, G.shaft_end, G.shoulder_dia, G.key_w, G.key_l, G.key_from_end, G.key_over) == (8.0, 21.5, 10.0, 3.0, 14.0, 2.0, 9.2)
    assert (G.bolt_pcd, G.bolt_depth, G.bore_dia, G.bore_depth, G.plate_t, G.plate_hole) == (35.0, 8.0, 22.0, 25.5, 4.7, 3.5)
    assert G.plate_sp == PARAMS.NEMA17_BOLT_SP and G.input_dia == PARAMS.NEMA17_SHAFT_DIA and G.bore_dia == PARAMS.NEMA17_PILOT_DIA
    assert G.boss_t + G.shoulder_t + 18.0 == G.shaft_end                                    # the drawing's 18 of Ø8
    assert G.keyway_depth() == pytest.approx(1.8)
    assert G.flange_t + G.band_t < G.length - G.bore_depth                                  # the bore in the square end
    assert G.channel < G.plate_sp / 2.0 - PARAMS.M3_SHCS.head_dia / 2.0                     # the motor's screw heads fit


@pytest.mark.slow
def test_the_build_matches_its_numbers():
    solids, volume, lo, size = LOCK
    shape = built.part(NAME)
    assert len(shape.solids()) == solids and shape.is_valid
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.05)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)


@pytest.mark.slow
def test_the_drawings_features():
    g = built.part(NAME)

    def solid(x, y, z):
        return is_inside(g, x, y, z)

    # the output face, the boss on it and the shaft's shoulder, the shaft to its end, the M3 hole in the end
    assert solid(15.0, 0.0, -0.1) and not solid(15.0, 0.0, 0.1)
    assert solid(0.0, 12.3, 1.0) and not solid(0.0, 12.7, 1.0) and not solid(0.0, 12.3, 2.1)
    assert solid(0.0, 4.9, 2.5) and not solid(0.0, 5.1, 2.5)
    assert solid(0.0, 3.9, 10.0) and not solid(0.0, 4.1, 10.0)
    assert solid(0.0, 3.0, G.shaft_end - 0.1) and not solid(0.0, 3.0, G.shaft_end + 0.1)
    assert not solid(0.0, 0.0, G.shaft_end - 0.1) and not solid(0.0, 0.0, G.shaft_end - G.end_tap_depth + 0.1)
    assert solid(0.0, 0.0, G.shaft_end - G.end_tap_depth - 0.1)
    # the keyway on +X: 3 wide, keyway_depth() deep, from key_from_end short of the end, 14 long
    z1, depth = G.shaft_end - G.key_from_end, G.keyway_depth()
    assert not solid(G.shaft_dia / 2.0 - depth / 2.0, 0.0, z1 - 7.0) and solid(G.shaft_dia / 2.0 - depth - 0.1, 0.0, z1 - 7.0)
    assert solid(3.5, 1.6, z1 - 7.0) and solid(3.5, 0.0, z1 + 0.1) and solid(3.5, 0.0, z1 - G.key_l - 0.1)
    assert solid(-3.5, 0.0, z1 - 7.0)                                                     # the shaft's -X side whole
    # the 4x M4 on Ø35, on the axes, bolt_depth deep
    r = G.bolt_pcd / 2.0
    for x, y in ((r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)):
        assert not solid(x, y, -G.bolt_depth + 0.1) and solid(x, y, -G.bolt_depth - 0.1)
        assert solid(x + G.bolt_hole_dia / 2.0 + 0.1, y, -1.0)
    # round output flange and band, both Ø42: their corners open (where the square end's would be)
    assert solid(20.9, 0.0, -5.0) and not solid(21.1, 0.0, -5.0) and not solid(15.5, 15.5, -5.0)
    assert solid(0.0, 20.9, -20.0) and not solid(15.5, 15.5, -20.0)
    # the square input end: its sides, its clipped corners, the corner channels down to the input plate
    z_sq = -(G.flange_t + G.band_t)
    assert solid(20.9, 0.0, -40.0) and not solid(21.1, 0.0, -40.0)
    assert not solid(20.5, 20.5, -G.length + 1.0) and solid(17.0, 19.0, -G.length + 1.0)   # 45 deg corner, 4 off
    assert not solid(15.0, 15.0, z_sq - 1.0) and not solid(15.0, 15.0, -G.length + G.plate_t + 0.1)
    assert solid(G.channel - 0.1, 15.0, z_sq - 5.0) and solid(14.0, 14.0, -G.length + G.plate_t - 0.1)
    # the plate's 4 holes on the 31 square
    h = G.plate_sp / 2.0
    for sx in (1, -1):
        for sy in (1, -1):
            assert not solid(sx * h, sy * h, -G.length + 1.0) and solid(sx * (h + G.plate_hole / 2.0 + 0.2), sy * h, -G.length + 1.0)
    # the input bore, the clamp's collar in it with the motor shaft's bore
    assert not solid(0.0, 10.9, -G.length + 1.0) and solid(0.0, 11.1, -G.length + 1.0)
    assert solid(0.0, 10.0, -G.length + G.bore_depth + 0.1) and not solid(0.0, 10.0, -G.length + G.bore_depth - 0.1)
    assert solid(0.0, 9.0, -40.0) and not solid(0.0, 9.0, -G.length + G.collar_from - 0.1)
    assert not solid(0.0, 0.0, -40.0) and solid(0.0, 2.6, -40.0)
    # the clamp's access holes: +Y in the +X face, -Y in the -X face
    z = -G.length + G.access_from_input
    assert not solid(19.0, G.access_off, z) and solid(19.0, -G.access_off, z)
    assert not solid(-19.0, -G.access_off, z) and solid(-19.0, G.access_off, z)
