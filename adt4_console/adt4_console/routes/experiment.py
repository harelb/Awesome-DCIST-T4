"""Experiment runs + data management (API.md §18)."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel

from ..experiment import export as X
from ..experiment.store import FAILURE_CATEGORIES, INTERVENTION_CATEGORIES, OUTCOMES, PROFILES
from . import services

router = APIRouter(prefix="/api")


def _exp(request: Request):
    svc = services(request).experiments
    if svc is None:
        raise HTTPException(503, "experiments are not available")
    return svc


def _call(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except KeyError as e:
        raise HTTPException(404, f"not found: {e}") from None
    except LookupError as e:
        raise HTTPException(409, str(e)) from None
    except ValueError as e:
        raise HTTPException(400, str(e)) from None


class ExperimentBody(BaseModel):
    name: str
    robots: list[str] = []
    bag_profile: str = "full"
    auto_record: bool = True
    notes: str = ""
    activate: bool = True


class ExperimentPatch(BaseModel):
    name: str | None = None
    notes: str | None = None
    robots: list[str] | None = None
    bag_profile: str | None = None
    auto_record: bool | None = None
    test_mode: bool | None = None


class ActivateBody(BaseModel):
    id: str | None = None


class PhaseBody(BaseModel):
    phase: str
    note: str = ""
    source: str | None = None


class OpenRunBody(BaseModel):
    labels: dict = {}
    instruction: str = ""
    robots: list[str] | None = None
    record: bool | None = None
    profile: str | None = None


class EndRunBody(BaseModel):
    stop_bags: bool = True
    outcome: dict | None = None


class PromptBody(BaseModel):
    action: str  # continue | dismiss


class RecordBody(BaseModel):
    enabled: bool


class MachinesBody(BaseModel):
    machines: list[str]


class InterventionBody(BaseModel):
    robot: str | None = None
    category: str = "other"
    note: str = ""


class RetractBody(BaseModel):
    seq: int
    reason: str = ""


class OutcomeBody(BaseModel):
    label: str
    failure_category: str | None = None
    notes: str = ""


class OffloadBody(BaseModel):
    target: str
    delete_after: bool = False


class TargetBody(BaseModel):
    spec: str
    name: str = ""


# ------------------------------------------------------------------ experiments
@router.get("/experiments")
def list_experiments(request: Request):
    svc = _exp(request)
    return {
        "experiments": svc.experiments(),
        "active": svc.exp["id"] if svc.exp else None,
        "profiles": list(PROFILES),
        "outcomes": list(OUTCOMES),
        "failure_categories": list(FAILURE_CATEGORIES),
        "intervention_categories": list(INTERVENTION_CATEGORIES),
    }


@router.post("/experiments")
def create_experiment(body: ExperimentBody, request: Request):
    return _call(_exp(request).create_experiment, body.name, body.robots, body.bag_profile, body.auto_record,
                 body.notes, body.activate)  # fmt: skip


@router.patch("/experiments/{exp_id}")
def update_experiment(exp_id: str, body: ExperimentPatch, request: Request):
    return _call(_exp(request).update_experiment, exp_id, body.model_dump(exclude_none=True))


@router.post("/experiments/activate")
def activate(body: ActivateBody, request: Request):
    return _call(_exp(request).activate, body.id)


@router.post("/experiments/phase")
def mark_phase(body: PhaseBody, request: Request):
    return _call(_exp(request).mark_phase, body.phase, body.note, body.source)


@router.get("/experiments/status")
def status(request: Request):
    return _exp(request).status()


@router.get("/experiments/recording")
def recording(request: Request):
    st = _exp(request).status()
    return {"recording": st["recording"], "logging": st["logging"], "test_mode": st["test_mode"],
            "run": (st["run"] or {}).get("id")}  # fmt: skip


# ------------------------------------------------------------------ the open run
@router.post("/experiments/run/open")
def open_run(body: OpenRunBody, request: Request):
    return _call(_exp(request).open_run, body.labels, body.instruction, body.robots, body.record, body.profile)


@router.post("/experiments/run/end")
def end_run(body: EndRunBody, request: Request):
    job = _call(_exp(request).end_run, body.stop_bags, body.outcome)
    return {"job_id": job.id} if hasattr(job, "id") else job


@router.post("/experiments/run/prompt")
def prompt(body: PromptBody, request: Request):
    if body.action not in ("continue", "dismiss"):
        raise HTTPException(400, "action is continue or dismiss (end: POST run/end)")
    return _exp(request).answer_prompt(body.action)


@router.post("/experiments/run/record")
def set_record(body: RecordBody, request: Request):
    return _call(_exp(request).set_record, body.enabled)


@router.post("/experiments/run/recorders/{machine}/{action}")
def recorder_action(machine: str, action: str, request: Request):
    svc = _exp(request)
    if action == "restart":
        return {"machines": _call(svc.restart_recorder, machine)}
    if action == "stop":
        return {"machines": _call(svc.stop_recording, [machine])}
    raise HTTPException(404, f"unknown recorder action {action}")


@router.post("/experiments/run/accept_missing")
def accept_missing(body: MachinesBody, request: Request):
    return _call(_exp(request).accept_missing, body.machines)


@router.post("/experiments/run/intervention")
def intervention(body: InterventionBody, request: Request):
    return _call(_exp(request).log_intervention, body.robot, body.category, body.note)


@router.post("/experiments/run/retract")
def retract(body: RetractBody, request: Request):
    return _call(_exp(request).delete_event, body.seq, body.reason)


# ------------------------------------------------------------------ runs
@router.get("/experiments/{exp_id}/runs")
def runs(exp_id: str, request: Request):
    return {"runs": _exp(request).runs(exp_id)}


@router.get("/experiments/{exp_id}/runs/{run_id}")
def run_detail(exp_id: str, run_id: str, request: Request):
    return _call(_exp(request).run_detail, exp_id, run_id)


@router.post("/experiments/{exp_id}/runs/{run_id}/outcome")
def outcome(exp_id: str, run_id: str, body: OutcomeBody, request: Request):
    return _call(_exp(request).set_outcome, exp_id, run_id, body.model_dump())


@router.post("/experiments/{exp_id}/runs/{run_id}/check")
def check(exp_id: str, run_id: str, request: Request):
    svc = _exp(request)
    run = _call(svc._load_run, exp_id, run_id)
    if run.get("status") in ("open", "recording"):
        raise HTTPException(409, "the run is still recording")
    job = svc.jobs.start("experiment-check", f"Check bags of {run_id}", lambda job: svc.check_bags(job, run=run))
    return {"job_id": job.id}


@router.post("/experiments/{exp_id}/runs/{run_id}/offload")
def offload(exp_id: str, run_id: str, body: OffloadBody, request: Request):
    job = _call(_exp(request).offload, exp_id, run_id, body.target, body.delete_after)
    return {"job_id": job.id}


@router.get("/experiments/{exp_id}/export.csv")
def export_csv(exp_id: str, request: Request, table: str = "runs", include_test: bool = False):
    runs = _exp(request).store.runs(exp_id)
    if table == "robots":
        text = X.to_csv(X.robot_rows(runs, include_test), X.ROBOT_COLUMNS)
    else:
        text = X.to_csv(X.run_rows(runs, include_test), X.RUN_COLUMNS)
    return PlainTextResponse(text, media_type="text/csv",
                             headers={"Content-Disposition": f'attachment; filename="{exp_id}_{table}.csv"'})  # fmt: skip


# ------------------------------------------------------------------ data management
@router.get("/data/targets")
def targets(request: Request):
    return {"targets": _exp(request).targets.all()}


@router.post("/data/targets")
def add_target(body: TargetBody, request: Request):
    return _call(_exp(request).targets.add_network, body.spec, body.name)


@router.delete("/data/targets/{target_id:path}")
def remove_target(target_id: str, request: Request):
    _exp(request).targets.remove(target_id)
    return {"ok": True}


@router.get("/data/disks")
def disks(request: Request):
    return {"machines": _exp(request).robot_disks()}
