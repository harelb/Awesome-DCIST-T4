// Spot lease indicator + "Take lease" (with a typed confirmation) and the
// "lease held by tablet — Take lease?" notice after a lease_not_owned refusal.
// Spot only: both render nothing for other platforms (lib/platform.ts).
import {useEffect, useState} from "react";
import {useRobots} from "../stores/robots";
import {errorText, postJSON} from "../lib/api";
import {consoleClockOffset} from "../lib/clock";
import {leaseFromGuards, leaseView, recentLeaseRefusal, shortOwner} from "../lib/lease";
import type {LeaseInfo} from "../types";
import {Badge, Modal, Spinner} from "../components/ui";
import {usePlatform} from "./usePlatform";

function consoleNow(): number {
  return Date.now() / 1000 + (consoleClockOffset() ?? 0);
}

/** Re-render every ``ms`` so staleness is re-evaluated without new messages. */
function useTick(ms = 5000): number {
  const [n, setN] = useState(0);
  useEffect(() => {
    const h = setInterval(() => setN(x => x + 1), ms);
    return () => clearInterval(h);
  }, [ms]);
  return n;
}

// When the operator last took each robot's lease (console seconds): hides the
// refusal notice that prompted it.
const tookAt: Record<string, number> = {};

export function useLease(robot: string | null): LeaseInfo | null {
  const fromPlan = useRobots(s => (robot ? s.plan[robot]?.lease : undefined));
  const guards = useRobots(s => (robot ? s.state[robot]?.guards : undefined));
  const platform = usePlatform(robot);
  if (!robot || platform !== "spot") return null;
  return fromPlan ?? leaseFromGuards(guards);
}

export function LeaseBadge({robot}: {robot: string}) {
  useTick();
  const v = leaseView(useLease(robot), consoleNow());
  return (
    <Badge tone={v.tone} title={v.title}>
      {v.label}
    </Badge>
  );
}

export function TakeLeaseButton({robot, owner, className = "btn btn-sm", onTaken}: {robot: string; owner?: string; className?: string; onTaken?: () => void}) {
  const [open, setOpen] = useState(false);
  const [typed, setTyped] = useState("");
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<{ok: boolean; text: string} | null>(null);
  const holder = owner ? shortOwner(owner) : "its current holder";

  const take = async () => {
    setBusy(true);
    setResult(null);
    try {
      const r = await postJSON<{ok: boolean; message: string}>(`api/robots/${encodeURIComponent(robot)}/take_lease`);
      setResult({ok: r.ok, text: r.message || (r.ok ? "lease taken" : "take failed")});
      if (r.ok) {
        tookAt[robot] = consoleNow();
        setOpen(false);
        setTyped("");
        onTaken?.();
      }
    } catch (e) {
      setResult({ok: false, text: errorText(e)});
    } finally {
      setBusy(false);
    }
  };

  return (
    <>
      <button className={className} onClick={() => setOpen(true)} title={`Forcibly take ${robot}'s Spot lease`}>
        Take lease
      </button>
      {result && !open && <span className={`small ${result.ok ? "tone-ok" : "tone-err"}`} role="status">{result.text}</span>}
      {open && (
        <Modal
          title={`Take ${robot}'s lease?`}
          tone="err"
          onClose={() => !busy && setOpen(false)}
          footer={
            <>
              <button className="btn" disabled={busy} onClick={() => setOpen(false)}>
                Cancel
              </button>
              <button className="btn btn-danger" disabled={busy || typed !== robot} onClick={() => void take()} data-testid="take-lease-yes">
                {busy && <Spinner />} Take lease
              </button>
            </>
          }
        >
          <p className="confirm-msg">
            The executor on {robot} will forcibly take the Spot lease from {holder}
            {owner && shortOwner(owner) !== owner ? ` (${owner})` : ""}. Whoever holds it now loses control at once, and the
            planner may then command the robot: a new or repaired plan can make it walk. Make sure the tablet operator knows.
          </p>
          <label className="field">
            <span className="field-label">
              Type <code className="mono req-text">{robot}</code> to confirm
            </span>
            <input
              className="input mono"
              autoFocus
              value={typed}
              onChange={e => setTyped(e.target.value)}
              onKeyDown={e => e.key === "Enter" && typed === robot && !busy && void take()}
              aria-label="Robot name to confirm"
            />
          </label>
          {result && !result.ok && <div className="inline-msg tone-err">{result.text}</div>}
        </Modal>
      )}
    </>
  );
}

/** Lease badge + Take lease button (button only while the lease is not ours). */
export function LeaseControl({robot}: {robot: string}) {
  const lease = useLease(robot);
  if (usePlatform(robot) !== "spot") return null;
  return (
    <span className="row gap-sm lease-control">
      <LeaseBadge robot={robot} />
      {!lease?.owned && <TakeLeaseButton robot={robot} owner={lease?.owner} className="btn btn-xs" />}
    </span>
  );
}

/** "hamilton: lease held by tablet — Take lease?" after the planner refused a
 *  plan with lease_not_owned (until the lease is ours or was taken since). */
export function LeaseRefusalNotice({robot}: {robot: string | null}) {
  useTick();
  const status = useRobots(s => (robot ? s.plan[robot]?.status : undefined));
  const lease = useLease(robot);
  const platform = usePlatform(robot);
  const [, force] = useState(0);
  if (!robot || platform !== "spot") return null;
  const refusal = recentLeaseRefusal(status, robot, consoleNow(), {since: tookAt[robot] ?? 0});
  if (!refusal || lease?.owned) return null;
  const owner = refusal.owner || lease?.owner || "";
  return (
    <div className="inline-msg tone-warn lease-refusal row gap-sm" role="alert">
      <span title="The planner refused the plan: the executor does not hold the Spot lease">
        {robot}: lease held by {owner ? shortOwner(owner) : "someone else"} — Take lease?
      </span>
      <TakeLeaseButton robot={robot} owner={owner} className="btn btn-xs" onTaken={() => force(x => x + 1)} />
    </div>
  );
}
