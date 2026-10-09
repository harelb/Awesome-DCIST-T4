"""API routers; each module exposes ``router``."""

from fastapi import HTTPException, Request


def services(request: Request):
    return request.app.state.services


def require_machine(svc, name: str) -> dict:
    m = svc.fleet.machine(name)
    if m is None:
        raise HTTPException(404, f"unknown machine {name}")
    return m
