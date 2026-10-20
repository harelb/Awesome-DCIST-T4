"""Fleet machine terminals (WebSocket) and tmux session stop/kill.

Terminal protocol: binary frames are keystrokes (client -> server) and terminal
output (server -> client); text frames are JSON control messages from the client,
``{"type": "resize", "cols": C, "rows": R}``.  The terminal is off unless the
console runs with ``--terminal``.
"""

from __future__ import annotations

import asyncio
import json
import logging

from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect

from .. import terminal as term
from . import require_machine, services

log = logging.getLogger(__name__)

router = APIRouter(prefix="/api")

# closed before accept: terminal disabled / unknown machine or session
CLOSE_DISABLED = 4403
CLOSE_BAD_TARGET = 4404

# Stop: after Ctrl-C, how long the panes get to exit before the session is killed
STOP_WAIT_S = 15.0
STOP_POLL_S = 1.0


@router.post("/fleet/{name}/tmux/{session}/{action}")
def tmux_session_action(name: str, session: str, action: str, request: Request):
    """``kill``: kill the session now.  ``stop``: a job that sends Ctrl-C to every
    pane, waits up to STOP_WAIT_S for the nodes to exit, then kills the session."""
    svc = services(request)
    m = require_machine(svc, name)
    if action not in ("stop", "kill"):
        raise HTTPException(404, f"unknown action {action}")
    try:
        cmd = term.session_command(action, session)
    except ValueError as e:
        raise HTTPException(400, str(e)) from e
    if action == "stop":

        def run(job):
            ok = term.stop_session(
                svc.helpers.ssh_cmd, m["user"], m["ip"], session, job.log,
                wait_s=STOP_WAIT_S, poll_s=STOP_POLL_S,
            )  # fmt: skip
            if svc.settings.fleet_poll:
                svc.fleet.request_status_refresh()
            return {"machine": name, "session": session} if ok else False

        job = svc.jobs.start("tmux_stop", f"Stop {session} on {name}", run)
        return {"ok": True, "machine": name, "session": session, "action": action, "job_id": job.id}
    rc, out, err = svc.helpers.ssh_cmd(m["user"], m["ip"], cmd, timeout=8)
    if rc != 0:
        raise HTTPException(502, f"{action} {session} on {name} failed: {err or out or rc}")
    if svc.settings.fleet_poll:
        svc.fleet.request_status_refresh()
    return {"ok": True, "machine": name, "session": session, "action": action}


@router.websocket("/fleet/{name}/terminal")
async def terminal(websocket: WebSocket, name: str):
    svc = websocket.app.state.services
    if not svc.settings.terminal:
        await websocket.close(code=CLOSE_DISABLED)
        return
    m = svc.fleet.machine(name)
    qs = websocket.query_params
    session = qs.get("session") or None
    if m is None or (session is not None and not term.valid_session(session)):
        await websocket.close(code=CLOSE_BAD_TARGET)
        return
    machine = {**m, "local": m.get("local") or m.get("ip") in svc.helpers.get_local_ips()}
    await websocket.accept()
    try:
        p = term.Pty(
            term.terminal_argv(machine, session),
            cols=int(qs.get("cols", 80)),
            rows=int(qs.get("rows", 24)),
        )
    except (OSError, ValueError) as e:
        await websocket.send_bytes(f"\r\nterminal failed: {e}\r\n".encode())
        await websocket.close()
        return
    log.info("terminal open: %s session=%s pid=%s", name, session, p.proc.pid)

    loop = asyncio.get_running_loop()
    out: asyncio.Queue[bytes] = asyncio.Queue()

    def on_readable():
        data = p.read()
        if data is None:
            return
        if not data:
            loop.remove_reader(p.fd)
        out.put_nowait(data)  # b"" = EOF

    loop.add_reader(p.fd, on_readable)

    async def pump_out():
        while True:
            data = await out.get()
            if not data:
                return
            await websocket.send_bytes(data)

    async def pump_in():
        while True:
            msg = await websocket.receive()
            if msg["type"] == "websocket.disconnect":
                return
            if msg.get("bytes"):
                p.write(msg["bytes"])
            elif msg.get("text"):
                try:
                    ctl = json.loads(msg["text"])
                except ValueError:
                    continue
                if ctl.get("type") == "resize":
                    p.resize(ctl.get("cols", 80), ctl.get("rows", 24))

    tasks = [asyncio.create_task(pump_out()), asyncio.create_task(pump_in())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    except WebSocketDisconnect:
        pass
    finally:
        for t in tasks:
            t.cancel()
        try:
            loop.remove_reader(p.fd)
        except (ValueError, OSError):
            pass
        await loop.run_in_executor(None, p.close)
        log.info("terminal closed: %s session=%s", name, session)
        try:
            await websocket.close()
        except RuntimeError:
            pass  # already closed
