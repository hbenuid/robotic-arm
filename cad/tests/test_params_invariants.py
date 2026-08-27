"""Locks on lib/params.py - add one whenever a shared dimension is introduced."""
import json
import math
import pathlib

from lib import params as p

MANIFEST = json.loads((pathlib.Path(__file__).resolve().parent.parent / "reference" / "manifest.json").read_text())["parts"]


def test_units():
    assert p.IN == 25.4
    assert 0 < p.NUDGE < 0.1


def test_gt2_belt_drive():
    assert p.GT2_PITCH == 2.0
    assert (p.GT2_PULLEY_90T_TEETH, p.GT2_PULLEY_20T_TEETH) == (90, 20)
    assert math.isclose(p.GT2_RATIO, 4.5)
    assert math.isclose(p.GT2_PULLEY_90T_PITCH_DIA, 90 * 2 / math.pi)
    # the printed 90T pulley's outer diameter must sit just under its pitch diameter + tooth
    assert p.GT2_PULLEY_90T_PITCH_DIA < MANIFEST["gt2_pulley_90t"]["bbox_size"][0] < p.GT2_PULLEY_90T_PITCH_DIA + 3


def test_pancake_envelope_tracks_reference():
    size = MANIFEST["nema17_pancake"]["bbox_size"]
    assert math.isclose(p.PANCAKE_BODY_W, size[0], abs_tol=0.05)
    assert math.isclose(p.PANCAKE_BODY_D, size[1], abs_tol=0.05)
    assert math.isclose(p.PANCAKE_BODY_H, size[2], abs_tol=0.05)
    assert p.PANCAKE_BODY_W < p.NEMA17_FACE + 0.5
    assert p.PANCAKE_MASS_G > 0


def test_gripper_rail_tracks_reference():
    size = sorted(MANIFEST["gripper_rail_6mm"]["bbox_size"])
    assert math.isclose(p.RAIL_DIA, size[0], abs_tol=0.01)
    assert math.isclose(p.RAIL_LEN, size[2], abs_tol=0.01)


def test_mg996r_fits_its_reference_envelope():
    size = sorted(MANIFEST["mg996r_servo"]["bbox_size"])
    assert p.MG996R_BODY_W <= size[0] + 0.5
    assert p.MG996R_BODY_L <= size[1] + 0.5
    assert p.MG996R_TAB_L <= size[2] + 1.5
    assert p.MG996R_MASS_G > 0
