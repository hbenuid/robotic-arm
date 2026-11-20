"""./cadtool stop - end this venv's idle workers on the machine's one server, and no other session's.

`cad server stop` stops the whole server, for every session on the machine, and the server has no verb for one
environment. This reads `cad server status --json` on stdin, where each worker names its Python, and ends this venv's
idle ones by their pid; the server clears a dead idle worker at its next grant, and the next build starts a fresh one.
It is what a change of build123d or OCP needs: a worker keeps the modules it loaded, and the same `cad` release reuses
it. A busy worker is left running and named (exit 1). No server running is no worker to end (exit 0); any other
refusal (a server of another release, which a `cad` command does not talk to) is said (exit 1).
"""
from __future__ import annotations

import json
import os
import signal
import sys


def split(status: dict, prefix: str) -> tuple[list[int], list[int]]:
    """(idle pids, busy pids) of the live workers running the environment at `prefix`."""
    prefix = os.path.realpath(prefix)
    idle, busy = [], []
    for worker in status.get("workers") or []:
        python = worker.get("python")
        if not python or not worker.get("alive"):
            continue
        if os.path.realpath(os.path.dirname(os.path.dirname(python))) != prefix:
            continue
        (busy if worker.get("busy") else idle).append(worker["pid"])
    return idle, busy


def refusal(status: dict) -> tuple[str, int] | None:
    """What a status that is no answer says, and the exit status: None for an answer (`"ok": true`)."""
    if status.get("ok") is True:
        return None
    error = str(status.get("error") or "no answer from `cad server status`")
    return (f"cadtool: {error} - no worker to end", 0) if "is not running" in error else (f"cadtool: {error}", 1)


def main() -> int:
    try:
        status = json.load(sys.stdin)
    except ValueError:
        status = {}
    refused = refusal(status)
    if refused is not None:
        print(refused[0])
        return refused[1]
    idle, busy = split(status, sys.prefix)
    for pid in idle:
        os.kill(pid, signal.SIGTERM)
    print(f"cadtool: {len(idle)} idle worker(s) of this venv ended" + (f", {len(busy)} busy left running: {busy}" if busy else ""))
    return 1 if busy else 0


if __name__ == "__main__":
    sys.exit(main())
