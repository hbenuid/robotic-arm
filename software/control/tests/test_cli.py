"""The CLIs on the fake boards: motor_control's menus, the launcher's arguments, command_sweep's safeguards."""

from __future__ import annotations

import io

import pytest

from robotic_arm import command_sweep, launcher, motor_control


class Terminal(io.StringIO):
    def isatty(self) -> bool:
        return True


def scripted(monkeypatch: pytest.MonkeyPatch, *answers: str) -> None:
    replies = iter(answers)
    monkeypatch.setattr("builtins.input", lambda _prompt="": next(replies))


# --- command_sweep ------------------------------------------------------------------------------------------------


def test_command_sweep_needs_the_bus_and_one_can_id_named(capsys):
    for argv in (["--interface", "virtual", "--channel", "x"], ["--can-id", "1", "--channel", "x"],
                 ["--interface", "virtual", "--channel", "x", "--can-id", "0"],
                 ["--interface", "virtual", "--channel", "x", "--can-id", "1", "--move-motor-deg", "720"]):
        with pytest.raises(SystemExit):
            command_sweep.parse_args(argv)


def test_command_sweep_refuses_motion_without_a_terminal(monkeypatch, channel, fake):
    monkeypatch.setattr("sys.stdin", io.StringIO("MOVE\n"))   # piped: no terminal
    assert command_sweep.main(["--interface", "virtual", "--channel", channel, "--can-id", "1", "--motion"]) == 1
    assert fake.received == []


def test_command_sweep_refuses_motion_unconfirmed(monkeypatch, channel, fake):
    monkeypatch.setattr("sys.stdin", Terminal())
    scripted(monkeypatch, "yes")
    assert command_sweep.main(["--interface", "virtual", "--channel", channel, "--can-id", "1", "--motion"]) == 1
    assert fake.received == []


def test_command_sweep_reads_without_moving(channel, fake):
    assert command_sweep.main(["--interface", "virtual", "--channel", channel, "--can-id", "2"]) == 0
    codes = {f.data[0] for f in fake.received}
    assert codes == {c for c, _, _ in command_sweep.READS}
    assert {f.can_id for f in fake.received} == {2}


def test_command_sweep_motion_on_the_fake(monkeypatch, channel, fake):
    monkeypatch.setattr("sys.stdin", Terminal())
    monkeypatch.setattr(command_sweep.time, "sleep", lambda _s: None)
    scripted(monkeypatch, "MOVE")
    assert command_sweep.main(["--interface", "virtual", "--channel", channel, "--can-id", "3", "--motion"]) == 0
    board = fake.boards[3]
    assert board.position == 0 and board.speed_rpm == 0 and not board.enabled
    assert max(abs(int.from_bytes(f.data[4:7], "big", signed=True)) for f in fake.received if f.data[0] in (0xF4, 0xF5)) == 4096


def test_the_sweep_shows_the_move_at_every_joint():
    text = command_sweep.what_it_moves(3, 90.0)
    assert "elbow_pitch" in text and "4.0 deg  <- config's joint" in text
    assert "base_yaw          15.0 deg" in text


# --- launcher ------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(("argv", "expected"), [
    (["--channel", "/dev/ttyACM9"], ["--channel", "/dev/ttyACM9"]),
    (["--interface", "socketcan", "--channel", "can0"], ["--interface", "socketcan", "--channel", "can0"]),
    (["--fake"], ["--fake"]),
    ([], ["--channel", "/dev/found", ]),
    (["--bitrate", "250000"], ["--channel", "/dev/found", "--bitrate", "250000"]),
])
def test_the_launcher_detects_a_port_only_when_it_needs_one(monkeypatch, argv, expected):
    calls = []
    monkeypatch.setattr("sys.argv", ["launcher", *argv])
    monkeypatch.setattr(launcher, "find_can_port", lambda: "/dev/found")
    monkeypatch.setattr(motor_control, "main", calls.append)
    launcher.main()
    assert calls == [expected]


# --- motor_control ---------------------------------------------------------------------------------------------------


def test_the_menus_on_fake_boards(monkeypatch, capsys):
    scripted(monkeypatch,
             "0", "3",                                  # select elbow_pitch
             "1", "1", "8", "9", "4", "b",              # read: encoder, version, power, status
             "4", "1", "b",                             # enable
             "2", "1", "", "", "2048", "b",             # absolute move to 2048 counts
             "1", "1", "b",                             # encoder again
             "3", "2", "b",                             # home status
             "0", "a", "7",                             # select all, emergency stop all
             "q")
    motor_control.main(["--fake"])
    out = capsys.readouterr().out
    assert "Selected: elbow_pitch" in out
    assert "Encoder: 0 counts" in out and "Encoder: 2048 counts" in out and "+2.00 deg of elbow_pitch" in out
    assert "Firmware 1.0.9" in out and "Supply 24.00 V" in out
    assert "Single-turn zero: success, homing: success" in out
    assert "Emergency stop sent to 6 boards." in out


def test_disabling_asks_first(monkeypatch, capsys):
    scripted(monkeypatch, "0", "1", "4", "2", "no", "b", "q")
    motor_control.main(["--fake"])
    assert "OK - Disabled" not in capsys.readouterr().out
