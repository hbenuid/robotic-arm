"""robotic_arm.motor_driver against the MKS SERVO42D/57D CAN manual V1.0.9.

Every frame written out in hex here is one of the manual's own examples (its section in the comment), sent to or from
CAN id 1, check byte last. The rest runs against the fake boards (robotic_arm.fake) or a hand-written reply.
"""

from __future__ import annotations

import time

import pytest
from conftest import REPLY_S, SILENT_S, answer_once

from robotic_arm.motor_driver import (
    PARAM_UNSUPPORTED,
    Checksum,
    MotorDriver,
    Power,
    checksum,
    emergency_stop_request,
    move_relative_axis_request,
    move_to_axis_request,
    speed_request,
)


def hexbytes(text: str) -> bytes:
    return bytes.fromhex(text)


@pytest.fixture
def motor(can):
    return MotorDriver(can, 0x01)


# --- frames: the manual's examples -------------------------------------------------------------------------------------


@pytest.mark.parametrize(("request_bytes", "frame"), [
    (bytes([0x30]), "30 31"),                                              # Part 4
    (bytes([0x31]), "31 32"),                                              # 11.3.3
    (bytes([0x82, 0x05]), "82 05 88"),                                     # 7.4
    (bytes([0x9E, 0x00]), "9E 00 9F"),                                     # 8.2
    (bytes([0x00, 0x83]), "00 83 84"),                                     # 5.1.10
    (hexbytes("9D 01 00 14 36 B0"), "9D 01 00 14 36 B0 99"),               # 5.4.2
    (hexbytes("94 00 00 20 00 02 58"), "94 00 00 20 00 02 58 0F"),         # 7.5
    (move_to_axis_request(0x4000, 600, 2), "F5 02 58 02 00 40 00 92"),     # 11.4.1
    (move_to_axis_request(-0x4000, 600, 2), "F5 02 58 02 FF C0 00 11"),    # 11.4.1
    (move_to_axis_request(0x28000, 300, 2), "F5 01 2C 02 02 80 00 A7"),    # 11.4.3
    (move_to_axis_request(0x7F8000, 300, 2), "F5 01 2C 02 7F 80 00 24"),   # 11.4.4
    (move_to_axis_request(0, 0, 4), "F5 00 00 04 00 00 00 FA"),            # 11.4.2
    (move_relative_axis_request(0x4000, 600, 2), "F4 02 58 02 00 40 00 91"),   # 11.3.1
    # 11.3.1 prints this one's check byte 09; its bytes sum to 0x310 (0x10) - a misprint: the +0x4000 twin and
    # 11.4.1's F5 version check out
    (move_relative_axis_request(-0x4000, 600, 2), "F4 02 58 02 FF C0 00 10"),
    (move_relative_axis_request(0x28000, 300, 2), "F4 01 2C 02 02 80 00 A6"),  # 11.3.3
    (speed_request(0, 320, 2), "F6 01 40 02 3A"),                          # 10.1
    (speed_request(1, 320, 2), "F6 81 40 02 BA"),                          # 10.1
    (speed_request(0, 300, 2), "F6 01 2C 02 26"),                          # 10.4
    (speed_request(0, 0, 2), "F6 00 00 02 F9"),                            # 10.2
    (speed_request(0, 0, 0), "F6 00 00 00 F7"),                            # 10.2
    (emergency_stop_request(), "F7 F8"),                                   # 11.3.3
])
def test_request_frames_match_the_manual(motor, request_bytes, frame):
    assert motor.encode(request_bytes) == hexbytes(frame)


def test_the_fixed_check_byte_is_0x6b():
    assert checksum(0x01, b"\x30", Checksum.FIXED) == 0x6B   # Part 4
    assert checksum(0x123, b"\x30") == checksum(0x23, b"\x30")   # the id's low byte is all that counts, mod 256


@pytest.mark.parametrize(("call", "frame"), [
    (lambda m: m.go_home(), "91 92"),                                      # 7.3.6, 7.4
    (lambda m: m.go_home(mode=0), "91 00 92"),                             # 7.3.6
    (lambda m: m.go_home(mode=1), "91 01 93"),                             # 7.7
    (lambda m: m.set_zero(), "92 93"),                                     # 11.4.3
    (lambda m: m.set_home_params(0, 0, 100, False, mode=0), "90 00 00 00 64 00 00 F5"),   # 7.4
    (lambda m: m.set_home_params(0, 0, 100, False, mode=1), "90 00 00 00 64 00 01 F6"),   # 7.5
    (lambda m: m.set_home_params(0, 0, 100, False, mode=2), "90 00 00 00 64 00 02 F7"),   # 7.6
    (lambda m: m.set_home_params(0, 0, 100, True, mode=0), "90 00 00 00 64 01 00 F6"),    # 8.2
    (lambda m: m.set_auto_report(0x31, 1000), "01 31 03 E8 1E"),           # 5.1.9
    (lambda m: m.read_param(0x83), "00 83 84"),                            # 5.1.10
    (lambda m: m.emergency_stop(), "F7 F8"),                               # 11.3.3
])
def test_commands_send_the_manuals_frames(fake, motor, call, frame):
    call(motor)
    assert fake.sent_to(1)[-1] == hexbytes(frame)


def test_frames_the_manual_has_no_example_for(fake, motor):
    motor.set_heartbeat(500)
    assert fake.sent_to(1)[-1] == motor.encode(hexbytes("98 00 00 01 F4"))   # 5.2.16: times, 4 bytes
    motor.emergency_stop(priority=True)
    assert fake.sent_to(1)[-1] == hexbytes("F7 01 F9")                      # 9.2.3: F7 01 + check
    motor.set_home_params(0, 1, 30, False)
    assert fake.sent_to(1)[-1] == motor.encode(hexbytes("90 00 01 00 1E 00"))   # no hm_mode byte


# --- replies: the manual's examples ------------------------------------------------------------------------------------


def test_the_encoder_reply_of_13_3(peer, motor):
    thread = answer_once(peer, hexbytes("31 FF FF FF 09 3C 96 0A"))
    assert motor.read_encoder(REPLY_S) == int.from_bytes(hexbytes("FF FF FF 09 3C 96"), "big", signed=True)
    thread.join()


@pytest.mark.parametrize(("raw", "power"), [
    ("38 57 83 00 0A 1D", Power(0x5783, 0x000A)),   # 5.1.13's figure: 22.403 V, 10 mA
    ("38 57 5D 01 8D 7B", Power(0x575D, 0x018D)),   # 22.365 V, 397 mA
])
def test_the_power_replies_of_5_1_13(peer, motor, raw, power):
    thread = answer_once(peer, hexbytes(raw))
    assert motor.read_power(REPLY_S) == power
    thread.join()


def test_the_auto_report_reply_of_5_1_9(peer, motor):
    thread = answer_once(peer, hexbytes("01 31 01 34"))
    assert motor.set_auto_report(0x31, 1000, REPLY_S) is True
    thread.join()


def test_the_read_param_reply_of_5_1_10_carries_the_params_code(peer, motor):
    thread = answer_once(peer, hexbytes("83 0C 80 10"))   # the working current, 3200 mA
    assert motor.read_param(0x83, REPLY_S) == hexbytes("0C 80")
    thread.join()


def test_the_auto_reports_after_5_1_9_read_like_replies(peer, motor):
    report = hexbytes("31 00 00 00 00 00 00 32")   # 5.1.9's figure: the board's own frame every 1000 ms
    peer.send(1, report)
    reply = motor.wait_reply(0x31, REPLY_S)
    assert reply.data == report[:-1] and reply.timestamp > 0


# --- replies: firmware differences and the fake ---------------------------------------------------------------------


@pytest.mark.parametrize(("body", "firmware"), [
    ("40 12 01 00 09 20", b"\x01\x00\x09"),   # three version bytes (the table's "firmVer[3]")
    ("40 12 01 09 20", b"\x01\x09"),          # two (its "Bytes 3-4")
])
def test_the_version_reply_takes_every_byte_between_byte2_and_the_boot_version(peer, motor, body, firmware):
    raw = hexbytes(body)
    thread = answer_once(peer, raw + bytes([checksum(1, raw)]))
    version = motor.read_version(REPLY_S)
    thread.join()
    assert version.calibrated and version.hardware == 2 and version.bootloader == 0x20
    assert version.firmware == firmware


@pytest.mark.parametrize(("body", "statuses"), [("3B 01 02", (1, 2)), ("3B 01", (1,))])
def test_the_home_status_keeps_every_status_byte(peer, motor, body, statuses):
    raw = hexbytes(body)
    thread = answer_once(peer, raw + bytes([checksum(1, raw)]))
    assert motor.read_home_status(REPLY_S) == statuses
    thread.join()


def test_reads_through_the_fake(fake, motor):
    board = fake.boards[1]
    board.position, board.speed_rpm, board.io = 0x7FF0, -300, 0b0101
    assert motor.read_encoder() == 0x7FF0
    assert motor.read_encoder_carry() == (1, 0x3FF0)   # 5.1.1: one turn past 0x3FF0
    assert motor.read_speed() == -300
    assert motor.read_io() == 0b0101
    assert motor.read_status() == 1
    assert motor.read_enabled() is False
    assert motor.read_stalled() is False
    assert motor.read_angle_error() == 0
    assert motor.read_version().firmware_text == "1.0.9"
    assert motor.read_param(0x82) == PARAM_UNSUPPORTED


def test_writes_through_the_fake(fake, motor):
    board = fake.boards[1]
    assert motor.move_to_axis(1000, 60, 2) is False   # disabled: the fake refuses
    assert motor.enable() and board.enabled
    assert motor.move_to_axis(1000, 60, 2) and board.position == 1000
    assert motor.move_relative_axis(-250, 60, 2) and board.position == 750
    assert motor.set_speed(1, 100, 2) and board.speed_rpm == -100
    assert motor.stop(acc=2) and board.speed_rpm == 0
    assert motor.go_home() == 1 and board.position == 0
    assert motor.wait_reply(0x91, REPLY_S).status == 2   # the second reply: complete
    assert motor.request(bytes([0x82, 0x05])).status == 1
    assert motor.read_param(0x82) == b"\x05"
    assert motor.calibrate(timeout=REPLY_S) == 1
    assert motor.set_heartbeat(500) and board.heartbeat_ms == 500
    assert motor.wait_stopped(REPLY_S)


def test_the_checksum_method_switches_both_ends(fake, motor):
    assert motor.set_checksum_method(Checksum.FIXED)
    assert motor.checksum is Checksum.FIXED and fake.boards[1].checksum is Checksum.FIXED
    assert fake.sent_to(1)[-1] == hexbytes("4C 01 4E")
    assert motor.read_encoder() == 0
    assert fake.sent_to(1)[-1] == hexbytes("31 6B")


# --- what a request does not take ------------------------------------------------------------------------------------------


def test_a_wrong_check_byte_is_no_reply(peer, motor):
    thread = answer_once(peer, hexbytes("F3 01 FF"))
    assert motor.enable(timeout=0.2) is False
    thread.join()


def test_another_boards_reply_is_not_this_ones(peer, motor):
    raw = hexbytes("F1 01")
    thread = answer_once(peer, raw + bytes([checksum(2, raw)]), can_id=1, reply_id=2)
    assert motor.read_status(0.2) is None
    thread.join()


def test_a_late_reply_to_an_earlier_request_is_dropped(fake, motor, can):
    fake.boards[1].position = 5
    motor.send(bytes([0x31]))                 # nobody waits for this one ...
    time.sleep(0.05)
    can.receive(0x99, timeout=0.01)           # ... its reply is read and kept
    fake.boards[1].position = 7
    assert motor.read_encoder() == 7          # the kept 5 is dropped, not taken


def test_a_silent_board_times_out(fake, motor):
    fake.boards[1].respond = False
    start = time.monotonic()
    assert motor.read_encoder(SILENT_S) is None
    assert motor.emergency_stop(timeout=SILENT_S) is False
    assert time.monotonic() - start < 1.0


@pytest.mark.parametrize("call", [
    lambda: speed_request(2, 100, 2),
    lambda: speed_request(0, 3001, 2),
    lambda: speed_request(0, 100, 256),
    lambda: move_to_axis_request(8_388_608, 100, 2),
    lambda: move_relative_axis_request(-8_388_608, 100, 2),
])
def test_out_of_range_arguments_are_refused(call):
    with pytest.raises(ValueError):
        call()


def test_a_board_id_is_1_to_2047_and_a_command_7_bytes(can):
    with pytest.raises(ValueError):
        MotorDriver(can, 0)   # the broadcast id: no board replies to it
    with pytest.raises(ValueError):
        MotorDriver(can, 1).encode(bytes(8))
