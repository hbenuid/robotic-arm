"""`uv run dashboard`: a page in the browser for driving and testing the MKS boards on the bus, served from this machine.

    uv run dashboard                          # finds the CANable (slcan), finds the boards, opens the page
    uv run dashboard --channel /dev/cu.usbmodem206E38534D4D1
    uv run dashboard --interface socketcan --channel can0
    uv run dashboard --fake                   # simulated boards that move in time - no hardware

It starts with --joint's board (at --can-id, or the joint's id in config.JOINTS), then scans ids 1-16 and adds every
board whose id is a joint's (--no-scan skips that); the page adds and removes motors too.

The server listens on 127.0.0.1 only. Every API call carries a token the server puts in the page it serves, and the
Host header must name this machine, so another site open in the browser cannot drive the motors. Ctrl+C - or SIGTERM
or SIGHUP (a killed background run, a closed terminal) - stops every moving motor (F7) and quits.
"""

from __future__ import annotations

import argparse
import json
import secrets
import signal
import sys
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from typing import Any
from urllib.parse import parse_qs, urlparse

from robotic_arm import config
from robotic_arm.can_interface import CANInterface
from robotic_arm.dashboard import routines
from robotic_arm.dashboard.bench import DEFAULT_JOINT_SPEED_CAP, Refused
from robotic_arm.dashboard.rig import Rig

DEFAULT_PORT = 8765
MAX_BODY = 64 * 1024
STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/ping.js": ("ping.js", "text/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}
CSP = ("default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; worker-src 'self'; "
       "img-src 'self' data:; base-uri 'none'; form-action 'none'; frame-ancestors 'none'")


def _int(value: Any) -> int:
    """An int from JSON: a number, or a string in decimal or 0x hex."""
    return int(value, 0) if isinstance(value, str) else int(value)


def dispatch(rig: Rig, action: str, b: dict[str, Any]) -> dict[str, Any]:
    """Run one page action - on motor b["motor"], or the active one; returns what the page shows. Raises Refused for a
    command the dashboard will not send."""
    match action:
        case "ping":
            return {}
        case "halt":
            confirmed = rig.halt_all()
            silent = [rig.motors[k].joint_config.name for k, ok in confirmed.items() if ok is None and k in rig.motors]
            failed = [rig.motors[k].joint_config.name for k, ok in confirmed.items() if ok is False and k in rig.motors]
            return {"message": f"STOP sent to {len(confirmed)} motor(s)"
                    + (f"; no confirmation from {', '.join(silent)}" if silent else "")
                    + (f"; F7 failed on {', '.join(failed)}" if failed else "") + ("" if silent or failed else ", confirmed")}
        case "select_motor":
            rig.set_active(str(b["key"]))
            return {}
        case "add_motor":
            key = rig.add_motor(str(b["joint"]), _int(b["can_id"]))
            if b.get("activate"):
                rig.set_active(key)
            return {"key": key}
        case "remove_motor":
            rig.remove_motor(str(b["key"]))
            return {}
        case "discover":
            added = rig.discover(_int(b.get("first", 1)), _int(b.get("last", 16)))
            return {"message": f"added {len(added)} motor(s)" if added else "no new boards with a joint's CAN id"}
        case "move_together":
            targets = {str(k): float(v) for k, v in dict(b["targets"]).items() if v not in (None, "")}
            return rig.move_together(targets, float(b.get("speed_pct", 50)), _int(b.get("acc", 2)),
                                     together=bool(b.get("together", True)))
    return dispatch_motor(rig, rig.bench(b.get("motor") or None), action, b)


def dispatch_motor(rig: Rig, bench: Any, action: str, b: dict[str, Any]) -> dict[str, Any]:
    """One motor's action."""
    match action:
        case "stop":
            bench.stop()
        case "enable":
            bench.enable(bool(b["on"]))
        case "select":
            bench.select(str(b["joint"]), _int(b["can_id"]))
        case "mode":
            bench.set_mode(str(b["mode"]))
        case "speed_limits":
            bench.set_speed_limits(*(None if b.get(k) in (None, "") else float(b[k]) for k in ("joint_speed", "bench_rpm")))
        case "profile":
            bench.set_profile(**{k: b.get(k) for k in ("kind", "curve", "ease_s", "rate_hz", "follow_acc")})
        case "poll_period":
            bench.set_poll_period(_int(b["ms"]))
        case "pid":
            gains = {k: None if b.get(k) in (None, "") else _int(b[k]) for k in ("kp", "ki", "kd", "kv")}
            return {"message": bench.write_pid(str(b["loop"]), **gains)}
        case "read_pid":
            return {"pid": bench.read_pid(str(b["loop"]))}
        case "limits_on":
            bench.set_limits_on(bool(b["on"]))
        case "limit":
            value = b.get("value")
            bench.set_limit(str(b["side"]), None if value in (None, "") else float(value), here=bool(b.get("here")))
        case "arm_limits":
            bench.use_arm_limits()
        case "sign":
            bench.set_sign(_int(b["sign"]))
        case "reference":
            bench.set_reference(float(b["deg"]))
        case "clear_reference":
            bench.clear_reference()
        case "move":
            bench.move_to(float(b["value"]), float(b["speed"]), _int(b["acc"]))
        case "step":
            bench.move_by(float(b["delta"]), float(b["speed"]), _int(b["acc"]))
        case "jog":
            return {"message": bench.jog(_int(b["direction"]), float(b["speed"]), _int(b["acc"]))}
        case "jog_release":
            bench.jog_release()
        case "run":
            bench.run(bool(b["cw"]), float(b["rpm"]), _int(b["acc"]))
        case "set_zero":
            bench.set_zero()
        case "release_stall":
            bench.release_stall()
        case "setting":
            value = b.get("value")
            return {"message": bench.write_setting(_int(b["code"]), None if value in (None, "") else _int(value))}
        case "read_settings":
            return {"settings": bench.read_settings()}
        case "raw":
            return bench.raw(str(b["hex"]))
        case "scan":
            found = bench.scan(_int(b.get("first", 1)), _int(b.get("last", 16)))
            ids = ", ".join(f"{f['can_id']} ({f['board']}, firmware {f['firmware']})" for f in found) or "none"
            cfg = bench.joint_config
            bench.add_finding(f"scan {b.get('first', 1)}..{b.get('last', 16)}: answering {ids}; config.JOINTS has "
                              f"{cfg.name} at {cfg.can_id}")
            return {"found": found}
        case "record":
            bench.set_recording(bool(b["on"]))
        case "routine":
            routines.start(bench, str(b["kind"]), dict(b.get("params") or {}))
        case "routine_cancel":
            if bench.routine is not None and bench.routine.active:
                bench.routine.cancel.set()
            bench.stop()
        case "routine_answer":
            if bench.routine is None:
                raise Refused("no test is waiting for an answer")
            bench.routine.answer(str(b["answer"]))
        case "log_polls":
            bench.worker.log_polls = bool(b["on"])
        case "clear_findings":
            bench.findings.clear()
        case _:
            raise Refused(f"unknown action {action!r}")
    return {}


def make_server(rig: Rig, host: str = "127.0.0.1", port: int = DEFAULT_PORT) -> ThreadingHTTPServer:
    """The HTTP server for `rig`; port 0 picks a free one (server.server_address)."""
    token = secrets.token_urlsafe(24)
    static = resources.files("robotic_arm.dashboard") / "static"

    class Handler(BaseHTTPRequestHandler):
        server_version = "robotic-arm-dashboard"
        protocol_version = "HTTP/1.1"

        def log_message(self, format: str, *args: Any) -> None:   # quiet: the page polls ten times a second
            pass

        def _host_ok(self) -> bool:
            address = self.server.server_address
            assert isinstance(address, tuple)   # an IPv4 server's (host, port)
            bound = address[1]
            return self.headers.get("Host", "") in {f"127.0.0.1:{bound}", f"localhost:{bound}"}

        def _token_ok(self) -> bool:
            return secrets.compare_digest(self.headers.get("X-Dashboard-Token", ""), token)

        def _send(self, status: int, body: bytes, ctype: str, extra: dict[str, str] | None = None) -> None:
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: Any, status: int = HTTPStatus.OK) -> None:
            self._send(status, json.dumps(obj, separators=(",", ":")).encode(), "application/json")

        def _refuse(self, status: int, text: str) -> None:
            self.close_connection = True   # a refused POST's body may be unread
            self._send(status, text.encode(), "text/plain; charset=utf-8")

        def do_GET(self) -> None:
            url = urlparse(self.path)
            if not self._host_ok():
                return self._refuse(HTTPStatus.FORBIDDEN, "the dashboard answers only at 127.0.0.1 / localhost")
            if url.path in STATIC:
                name, ctype = STATIC[url.path]
                body = (static / name).read_bytes()
                if name == "index.html":
                    body = body.replace(b"__TOKEN__", token.encode())
                return self._send(HTTPStatus.OK, body, ctype, {"Content-Security-Policy": CSP})
            if not url.path.startswith("/api/"):
                return self._refuse(HTTPStatus.NOT_FOUND, "not found")
            if not self._token_ok():
                return self._refuse(HTTPStatus.FORBIDDEN, "missing or wrong token: reload the page")
            rig.touch()
            q = {k: v[-1] for k, v in parse_qs(url.query).items()}
            if url.path == "/api/state":
                try:
                    after = (int(q.get("samples", 0)), int(q.get("log", 0)), int(q.get("notices", 0)))
                except ValueError:
                    return self._refuse(HTTPStatus.BAD_REQUEST, "bad query")
                return self._json(rig.state(*after, active=q.get("active")))
            bench = rig.bench()
            if url.path == "/api/record.csv":
                return self._send(HTTPStatus.OK, bench.recording_csv().encode(), "text/csv; charset=utf-8")
            if url.path == "/api/routine.csv":
                if bench.routine is None:
                    return self._refuse(HTTPStatus.NOT_FOUND, "no test has run")
                return self._send(HTTPStatus.OK, bench.routine.csv().encode(), "text/csv; charset=utf-8")
            return self._refuse(HTTPStatus.NOT_FOUND, "not found")

        def do_POST(self) -> None:
            if not self._host_ok() or not self._token_ok():
                return self._refuse(HTTPStatus.FORBIDDEN, "forbidden")
            if urlparse(self.path).path != "/api/cmd":
                return self._refuse(HTTPStatus.NOT_FOUND, "not found")
            length = int(self.headers.get("Content-Length") or 0)
            if not 0 < length <= MAX_BODY:
                return self._refuse(HTTPStatus.BAD_REQUEST, "bad body")
            rig.touch()
            try:
                body = json.loads(self.rfile.read(length))
                result = dispatch(rig, str(body.get("action")), body)
            except Refused as e:
                return self._json({"ok": False, "error": str(e)})
            except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
                return self._json({"ok": False, "error": f"bad request: {type(e).__name__}: {e}"}, HTTPStatus.BAD_REQUEST)
            except Exception as e:  # noqa: BLE001 - shown on the page rather than a dropped connection
                return self._json({"ok": False, "error": f"internal error: {e!r}"}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return self._json({"ok": True, **result})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def fake_boards(channel: str):  # -> FakeBoards
    """Simulated boards for --fake: one per joint at its config id, moving in time, its home switch's zone running
    20 deg past home_deg toward the joint's lower end (the shaft's zero is the joint at 0 deg)."""
    from robotic_arm.fake import FakeBoard, FakeBoards
    from robotic_arm.joint import Joint

    boards = []
    for cfg in config.JOINTS:
        if cfg.can_id is None:
            continue
        joint = Joint(cfg)
        edge = round(cfg.sign * cfg.home_deg * joint.counts_per_deg)
        far = round(cfg.sign * (cfg.home_deg - 20) * joint.counts_per_deg)
        boards.append(FakeBoard(cfg.can_id, simulate=True, switch_zone=(min(edge, far), max(edge, far)), io=1))
    return FakeBoards(channel, boards)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="dashboard", description="robotic-arm MKS motor dashboard (a local web page)")
    parser.add_argument("--channel", help="CAN channel; default: the CANable found (slcan), or can0's like for socketcan")
    parser.add_argument("--interface", default=config.CAN_INTERFACE,
                        help=f"python-can interface: slcan, socketcan ... (default: {config.CAN_INTERFACE})")
    parser.add_argument("--bitrate", type=int, default=config.CAN_BITRATE, help=f"default {config.CAN_BITRATE}")
    parser.add_argument("--fake", action="store_true", help="simulated boards that move in time, no hardware")
    parser.add_argument("--joint", default=config.JOINTS[0].name, choices=[j.name for j in config.JOINTS],
                        help="the joint the page starts on (it can be changed there)")
    parser.add_argument("--can-id", type=lambda s: int(s, 0), help="the board's CAN id (default: the joint's in config)")
    parser.add_argument("--max-joint-speed", type=float, default=DEFAULT_JOINT_SPEED_CAP,
                        help=f"joint mode's speed cap, deg/s (default {DEFAULT_JOINT_SPEED_CAP:g}); the page cannot pass it")
    parser.add_argument("--priority-estop", action="store_true",
                        help="STOP sends V1.0.9's immediate form (F7 01) - once the boards are known to run it")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"the page's port (default {DEFAULT_PORT})")
    parser.add_argument("--no-browser", action="store_true", help="do not open the page")
    parser.add_argument("--no-scan", action="store_true",
                        help="only --joint's board: do not look for the others at start (the page can add them)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    fake = None
    if args.fake:
        args.interface, args.channel = "virtual", "dashboard-fake"
        fake = fake_boards(args.channel)
    elif args.channel is None:
        if args.interface != "slcan":
            sys.exit(f"--channel is needed with --interface {args.interface}")
        from robotic_arm.launcher import find_can_port

        args.channel = find_can_port()
        print(f"Auto-detected CAN port: {args.channel}")
    bitrate = None if args.interface in ("socketcan", "virtual") else args.bitrate
    try:
        can = CANInterface(args.channel, args.interface, bitrate)
    except Exception as e:  # noqa: BLE001 - serial, python-can or OS: any failure to open ends the run
        sys.exit(f"Failed to open the CAN bus: {e}")
    label = f"{args.channel} ({args.interface}{f', {bitrate} bps' if bitrate else ''})" + (" - SIMULATED" if fake else "")
    rig = Rig(can, max_joint_speed=args.max_joint_speed, priority_estop=args.priority_estop, label=label)
    first = next(j for j in config.JOINTS if j.name == args.joint)
    rig.add_motor(args.joint, args.can_id or first.can_id or 1)
    try:
        server = make_server(rig, port=args.port)
    except OSError as e:
        rig.close()
        can.close()
        sys.exit(f"Cannot listen on 127.0.0.1:{args.port} ({e}): is a dashboard running already? --port picks another")
    print(f"Connected to {label}")
    if not args.no_scan:
        added = rig.discover(1, 16)
        names = [f"{rig.motors[k].joint_config.name} (id {rig.motors[k].can_id})" for k in rig.ordered()]
        print(f"Motors: {', '.join(names)}" + (f" - {len(added)} found on the bus" if added else ""))
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"Dashboard: {url}   (Ctrl+C stops every moving motor and quits)", flush=True)

    def on_terminate(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt   # SIGTERM (a killed background run, a closed terminal) stops the motor like Ctrl+C

    signal.signal(signal.SIGTERM, on_terminate)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, on_terminate)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nStopping ...")
    finally:
        if any(b.motion is not None or b.readout.status not in (None, 1) for b in rig.motors.values()):
            confirmed = rig.halt_all("the dashboard is closing")
            print("STOP:", "confirmed" if all(confirmed.values()) else "sent")
        server.server_close()
        rig.close()
        can.close()
        if fake is not None:
            fake.close()
        print("CAN bus closed.")


if __name__ == "__main__":
    main()
