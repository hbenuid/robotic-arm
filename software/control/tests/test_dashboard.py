"""The dashboard on simulated boards (fake.FakeBoard(simulate=True)): its checks, its safety rules, its tests, its
HTTP server. Nothing here reaches hardware."""

from __future__ import annotations

import http.client
import json
import threading
import time
from collections.abc import Callable, Iterator

import pytest

from robotic_arm.can_interface import CANInterface
from robotic_arm.dashboard import routines, server
from robotic_arm.dashboard.bench import Bench, Refused
from robotic_arm.dashboard.bus import describe
from robotic_arm.fake import FakeBoard, FakeBoards
from robotic_arm.motor_driver import Cmd, MotorDriver

ELBOW = 3


def wait_until(pred: Callable[[], object], timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not pred():
        if time.monotonic() > deadline:
            raise AssertionError("condition not met in time")
        time.sleep(0.01)


@pytest.fixture
def sim(channel: str) -> Iterator[FakeBoards]:
    with server.fake_boards(channel) as boards:
        yield boards


@pytest.fixture
def bench(channel: str, sim: FakeBoards) -> Iterator[Bench]:
    can = CANInterface(channel, "virtual", bitrate=None)
    b = Bench(can, joint="elbow_pitch", max_joint_speed=60.0)
    wait_until(lambda: b.readout.counts is not None)
    yield b
    b.close()
    can.close()


def frames(sim: FakeBoards, code: int, can_id: int = ELBOW) -> list[bytes]:
    return [f for f in sim.sent_to(can_id) if f[0] == code]


def run(bench: Bench, kind: str, answers: list[str] | None = None, timeout: float = 15.0, **params) -> routines.Routine:
    r = routines.start(bench, kind, params)
    answers = list(answers or [])
    deadline = time.monotonic() + timeout
    while r.active:
        if r.question is not None and answers:
            r.answer(answers.pop(0))
        assert time.monotonic() < deadline, f"{kind} still {r.status}: {r.progress}"
        time.sleep(0.02)
    return r


def bench_mode(bench: Bench, rpm: float = 600) -> None:
    bench.set_mode("bench")
    bench.set_speed_limits(bench_rpm=rpm)
    bench.enable(True)


# --- the simulated boards -------------------------------------------------------------------------------------------


def test_a_simulated_move_takes_time_and_answers_twice(channel):
    with FakeBoards(channel, [FakeBoard(1, simulate=True)]) as fake, CANInterface(channel, "virtual", bitrate=None) as can:
        motor = MotorDriver(can, 1)
        assert motor.enable(True)
        assert motor.move_to_axis(0x4000, 600, 200)    # one motor turn at 600 RPM: about 0.1 s plus the ramps
        assert 0 <= fake.boards[1].position < 0x4000
        done = motor.wait_reply(Cmd.MOVE_TO_AXIS, 2.0)
        assert done is not None and done.status == 2
        assert motor.read_encoder() == 0x4000
        assert motor.move_to_axis(100 * 0x4000, 300, 100)
        time.sleep(0.2)
        assert motor.read_speed() > 0
        assert motor.move_to_axis(0, 0, 0)               # 11.4.2: speed 0, acc 0 stops at once
        assert motor.wait_stopped(1.0)
        assert 0x4000 < motor.read_encoder() < 100 * 0x4000


def test_the_switch_zone_reads_on_in1_and_homing_zeroes_at_its_edge(channel):
    with FakeBoards(channel, [FakeBoard(1, simulate=True, switch_zone=(-1000, -500), io=1)]), \
            CANInterface(channel, "virtual", bitrate=None) as can:
        motor = MotorDriver(can, 1)
        motor.enable(True)
        assert motor.read_io() & 1 == 1
        motor.move_to_axis(-700, 600, 0)
        motor.wait_reply(Cmd.MOVE_TO_AXIS, 2.0)
        time.sleep(0.02)
        assert motor.read_io() & 1 == 0                  # inside: low, as a KY-003 pulls it
        assert motor.go_home() == 1
        assert motor.read_encoder() == 0                 # the zone's edge is the new zero ...
        motor.move_to_axis(-100, 600, 0)
        motor.wait_reply(Cmd.MOVE_TO_AXIS, 2.0)
        time.sleep(0.02)
        assert motor.read_io() & 1 == 0                  # ... and the zone did not move with it


# --- checks before sending --------------------------------------------------------------------------------------------


def test_joint_mode_checks_every_move_before_sending(bench, sim):
    bench.check(150, 5, 2)                               # no reference, no limit set: open (from the encoder's zero)
    bench.set_reference(0.0)
    bench.enable(True)
    with pytest.raises(Refused, match="outside the limits"):
        bench.move_to(150, 5, 2)                         # with a reference the arm's 144 applies
    with pytest.raises(Refused, match="cap"):
        bench.move_to(5, 61, 2)                          # past the 60 deg/s cap
    bench.set_speed_limits(joint_speed=10)
    with pytest.raises(Refused, match="cap"):
        bench.move_to(5, 11, 2)
    with pytest.raises(Refused):
        bench.set_speed_limits(joint_speed=61)           # past the dashboard's cap
    bench.set_limit("max", 10.0)
    with pytest.raises(Refused, match="outside the limits"):
        bench.move_to(15, 5, 2)                          # past your own max
    assert frames(sim, Cmd.MOVE_TO_AXIS) == []
    bench.move_to(2, 10, 2)
    sent = frames(sim, Cmd.MOVE_TO_AXIS)[-1]
    assert int.from_bytes(sent[1:3], "big") == 37        # 10 deg/s is 37.5 RPM: rounded DOWN, never past the cap
    assert int.from_bytes(sent[4:7], "big", signed=True) == round(2 * bench.joint().counts_per_deg)


def test_limits_off_means_free_movement(bench, sim):
    bench.set_reference(0.0)
    with pytest.raises(Refused, match="outside the limits"):
        bench.check(150, 5, 2)
    bench.set_limits_on(False)
    assert bench.check(150, 5, 2)[0] == round(150 * bench.joint().counts_per_deg)
    assert bench.window() == (None, None)
    with pytest.raises(Refused, match="cap"):
        bench.check(150, 61, 2)                          # the speed cap stays
    bench.set_limits_on(True)
    with pytest.raises(Refused, match="outside the limits"):
        bench.check(150, 5, 2)


def test_limits_set_here_follow_the_position_not_the_units(bench, sim):
    bench_mode(bench)
    seq = bench.move_to(90, 600, 250)
    bench.wait_motion_end(seq, 2.0, threading.Event())
    wait_until(lambda: bench.readout.counts == 0x1000)
    bench.set_limit("max", here=True)                    # 90 deg of the motor
    bench.set_limit("min", -45.0)
    assert bench.limit_view() == {"on": True, "min": -45.0, "min_src": "yours", "max": 90.0, "max_src": "yours"}
    with pytest.raises(Refused, match="outside the limits"):
        bench.check(100, 60, 2)
    with pytest.raises(Refused, match="at the max limit"):
        bench.jog(1, 60, 2)                              # at the max: no jog further that way
    with pytest.raises(Refused, match="below"):
        bench.set_limit("min", 120.0)                    # min above max
    bench.set_mode("joint")                              # the same physical limits in joint units (sign -1 flips them)
    bench.set_sign(-1)
    view = bench.limit_view()
    cpd = bench.joint().counts_per_deg
    assert view["min"] == pytest.approx(-0x1000 / cpd) and view["max"] == pytest.approx(0x800 / cpd)   # sign -1
    bench.set_zero()                                     # the encoder's zero moves: the limits move with it
    assert bench.limits.count_hi == 0 and bench.limits.count_lo == round(-45 / 360 * 0x4000) - 0x1000
    bench.homed()                                        # homing moves the zero by an unknown amount: cleared
    assert bench.limits.count_lo is None and bench.limits.count_hi is None


def test_bench_mode_speed_cap(bench, sim):
    bench.set_mode("bench")
    bench.enable(True)
    with pytest.raises(Refused, match="cap"):
        bench.move_to(90, 100, 2)                        # the 60 RPM default
    with pytest.raises(Refused):
        bench.set_speed_limits(bench_rpm=1500)           # the manual's 1000 RPM ceiling
    bench.set_speed_limits(bench_rpm=600)
    seq = bench.move_to(90, 600, 250)
    assert bench.wait_motion_end(seq, 2.0, threading.Event()) == 2
    assert bench.fresh_counts() == round(90 / 360 * 0x4000)


def test_a_constant_speed_run_is_bench_modes_only_and_stops_at_a_limit(bench, sim):
    bench.set_reference(0)
    with pytest.raises(Refused, match="bench mode"):
        bench.run(False, 30, 2)
    bench_mode(bench)
    bench.set_limit("min", -90.0)
    bench.set_limit("max", 90.0)
    bench.run(False, 300, 200)                           # CCW: the fake counts up
    wait_until(lambda: bench.motion is None and bench.readout.status == 1, 3.0)
    stops = [f for f in frames(sim, Cmd.SPEED) if f[1] & 0x0F == 0 and f[2] == 0]
    assert len(stops) == 1                               # the limit's ramped F6 stop, sent once
    assert sum("reached a limit" in n["text"] for n in bench.notices) == 1
    assert 90 < bench.readout.counts * 360 / 0x4000 < 200


def test_the_raw_box_refuses_what_moves(bench, sim):
    for text in ("F5 00 10 02 00 00 10", "f6 01 00 02", "91", "80 00"):
        with pytest.raises(Refused, match="moves the motor"):
            bench.raw(text)
    assert not any(f[0] in (0xF5, 0xF6, 0x91, 0x80) for f in sim.sent_to(ELBOW))
    assert bench.raw("40")["reply"].startswith("40 ")


# --- the safety rules ----------------------------------------------------------------------------------------------------


def test_a_jog_stops_when_its_keepalives_stop(bench, sim):
    bench_mode(bench, 300)                               # no limits set: the jog's target is its lead ahead
    bench.jog(1, 300, 200)
    time.sleep(0.15)
    assert bench.jog(1, 300, 200) == "jogging"           # a keep-alive, then none
    wait_until(lambda: any(f[1:3] == b"\x00\x00" for f in frames(sim, Cmd.MOVE_TO_AXIS)), 1.5)
    wait_until(lambda: bench.readout.status == 1 and bench.readout.speed_rpm == 0, 3.0)
    assert 0 < bench.readout.counts < 10 * 0x4000        # stopped well short of its 10-turn lead


def test_a_long_jog_moves_its_target_on(bench, sim, monkeypatch):
    from robotic_arm.dashboard import bench as bench_module

    monkeypatch.setattr(bench_module, "JOG_LEAD_S", 0.2)   # a short lead: the keep-alives must move it on
    bench_mode(bench, 300)
    lead = 300 * 0x4000 / 60 * 0.2                         # acc 255: braking is short, the lead sets the target
    end = time.monotonic() + 0.9
    bench.jog(1, 300, 255)
    while time.monotonic() < end:
        time.sleep(0.1)
        bench.jog(1, 300, 255)
    assert bench.readout.counts > 1.5 * lead              # well past the first target
    targets = [int.from_bytes(f[4:7], "big", signed=True) for f in frames(sim, Cmd.MOVE_TO_AXIS) if f[1:3] != b"\x00\x00"]
    assert len(targets) >= 3 and targets == sorted(targets)
    bench.jog_release()
    wait_until(lambda: bench.readout.status == 1 and bench.readout.speed_rpm == 0, 3.0)


def test_the_page_going_silent_halts_motion(bench, sim):
    bench_mode(bench, 60)
    bench.move_to(300, 60, 2)
    time.sleep(0.2)
    bench._last_page = time.monotonic() - 10
    wait_until(lambda: frames(sim, Cmd.EMERGENCY_STOP), 1.0)
    wait_until(lambda: bench.readout.status == 1, 1.0)
    assert bench.readout.counts < round(300 / 360 * 0x4000)
    assert any("silent" in n["text"] for n in bench.notices)


def test_stop_sends_f7_ahead_of_the_position_stop(bench, sim):
    assert bench.halt() is True
    codes = [f[0] for f in sim.sent_to(ELBOW)]
    i = len(codes) - 1 - codes[::-1].index(Cmd.EMERGENCY_STOP)
    assert Cmd.MOVE_TO_AXIS in codes[i + 1:]             # F7 went out first, then the F5 stop


def test_a_board_back_from_silence_loses_the_reference(bench, sim):
    bench.set_reference(10.0)
    sim.boards[ELBOW].respond = False
    wait_until(lambda: not bench._online, 2.5)
    assert bench.zero_deg is not None
    sim.boards[ELBOW].respond = True
    wait_until(lambda: bench._online, 2.0)
    wait_until(lambda: bench.zero_deg is None, 1.0)


def test_cancelling_a_test_stops_the_motor(bench, sim):
    bench_mode(bench, 60)
    r = routines.start(bench, "repeat", {"a": 0, "b": 300, "cycles": 3, "speed": 60, "acc": 2, "dwell": 0})
    wait_until(lambda: bench.readout.speed_rpm, 2.0)
    r.cancel.set()
    wait_until(lambda: not r.active, 2.0)
    assert r.status == "cancelled"
    wait_until(lambda: any(f[1:3] == b"\x00\x00" for f in frames(sim, Cmd.MOVE_TO_AXIS)), 1.0)
    wait_until(lambda: bench.readout.status == 1, 3.0)


# --- settings, scan ---------------------------------------------------------------------------------------------------


def test_settings_are_range_checked_and_the_can_id_followed(bench, sim):
    assert "1200" in bench.write_setting(0x83, 1200)
    assert sim.boards[ELBOW].params[0x83] == (1200).to_bytes(2, "big")
    with pytest.raises(Refused):
        bench.write_setting(0x83, 5000)
    with pytest.raises(Refused):
        bench.write_setting(0x8A, 1)                     # the bitrate is not one the page writes
    bench.write_setting(0x8B, 7)
    assert bench.can_id == 7 and bench.worker.poll_id == 7


def test_the_scan_finds_every_board(bench):
    found = bench.scan(1, 10)
    assert [f["can_id"] for f in found] == [1, 2, 3, 4, 5, 6]
    assert found[0]["firmware"] == "1.0.9"


# --- the tests (routines) ---------------------------------------------------------------------------------------------


def test_repeat_moves(bench):
    bench_mode(bench)
    r = run(bench, "repeat", a=0, b=90, cycles=2, speed=600, acc=250, dwell=0)
    assert r.status == "done", r.error
    assert [row[1] for row in r.rows] == ["B", "A", "B", "A"]
    assert all(abs(row[4]) < 0.05 for row in r.rows)
    assert any(line.startswith("at A") for line in r.summary)


def test_the_direction_check_measures_the_sign(bench):
    bench.enable(True)
    r = run(bench, "direction", ["Yes, go", "Negative: toward its home switch"], amount=1, speed=20)
    assert r.status == "done", r.error
    assert bench.sign == -1
    assert any("sign -1" in f for f in bench.findings)
    assert any('JointConfig("elbow_pitch", 0x03, 1 / 22.5, -1' in line for line in r.summary)


def test_homing_references_joint_mode_and_the_switch_zone_finds_home(bench):
    bench.enable(True)
    r = run(bench, "home", rpm=30, direction=0, trigger=0, end_limit=0, hm_mode=0)
    assert r.status == "done", r.error
    assert bench.zero_deg == -41.0
    r = run(bench, "switch", a=-36, b=-47, speed=20, acc=250)
    assert r.status == "done", r.error
    on = [row[2] for row in r.rows if row[1].startswith("on")]
    assert on and abs(on[0] - -41.0) < 0.5
    assert any("home_deg" in line for line in r.summary)


def test_a_step_is_measured(bench):
    bench_mode(bench)
    r = run(bench, "step", step=90, speed=600, acc=250, duration=0.6)
    assert r.status == "done", r.error
    assert len(r.rows) > 20
    assert any(line.startswith("rise 10-90 %") and "n/a" not in line for line in r.summary)


def test_a_speed_sweep(bench):
    bench_mode(bench)
    r = run(bench, "sweep", a=0, b=360, speeds="300 600", acc=250)
    assert r.status == "done", r.error
    assert [row[0] for row in r.rows] == [300, 600]
    assert all(row[2] is not None for row in r.rows)     # the simulated 0x38 current


def test_the_firmware_check(bench):
    r = run(bench, "firmware")
    assert r.status == "done", r.error
    assert "1.0.9" in r.summary[0]


def test_calibration_is_refused_in_joint_mode(bench):
    r = run(bench, "calibrate")
    assert r.status == "failed" and "bench mode" in r.error


# --- the HTTP server ------------------------------------------------------------------------------------------------------


def test_the_server_needs_its_token_and_a_local_host(bench):
    srv = server.make_server(bench, port=0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    try:
        def get(path: str, headers: dict[str, str] | None = None) -> tuple[int, bytes, http.client.HTTPResponse]:
            conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            conn.request("GET", path, headers=headers or {})
            res = conn.getresponse()
            return res.status, res.read(), res

        status, page, res = get("/")
        assert status == 200 and "default-src 'none'" in res.getheader("Content-Security-Policy")
        token = page.decode().split('name="dashboard-token" content="')[1].split('"')[0]
        assert len(token) > 20
        assert get("/api/state")[0] == 403
        assert get("/api/state", {"X-Dashboard-Token": "wrong"})[0] == 403
        assert get("/", {"Host": "evil.example:80"})[0] == 403
        status, body, _ = get("/api/state", {"X-Dashboard-Token": token})
        assert status == 200 and json.loads(body)["joint"] == "elbow_pitch"

        def post(body: dict) -> dict:
            conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            conn.request("POST", "/api/cmd", body=json.dumps(body),
                         headers={"X-Dashboard-Token": token, "Content-Type": "application/json"})
            return json.loads(conn.getresponse().read())

        assert post({"action": "ping"}) == {"ok": True}
        refused = post({"action": "move", "value": 1, "speed": 100, "acc": 2})
        assert refused["ok"] is False and "cap" in refused["error"]
        assert post({"action": "limit", "side": "max", "value": 5}) == {"ok": True}
        assert post({"action": "limits_on", "on": False}) == {"ok": True}
        assert bench.limit_view()["on"] is False
        assert post({"action": "nonsense"})["ok"] is False
    finally:
        srv.shutdown()
        srv.server_close()


def test_the_log_describes_motion_frames():
    assert describe("tx", bytes([0xF5, 0x00, 0x26, 0x02, 0x00, 0x04, 0x00])) == \
        "move to axis: to 1024 counts at 38 RPM, acc 2"
    assert describe("tx", bytes([0xF5, 0, 0, 4, 0, 0, 0])) == "move to axis: stop (ramp 4)"
    assert describe("tx", bytes([0xF6, 0x80, 0x1E, 0x02])) == "speed: CW at 30 RPM, acc 2"
    assert describe("rx", bytes([0xF5, 0x02])) == "reply F5 (move to axis): 02"
