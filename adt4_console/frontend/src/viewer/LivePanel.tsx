import {useEffect, useState} from "react";
import {frameAlignOf, useRobots} from "../stores/robots";
import {AlignStatus} from "./RobotLegend";
import {useViewer} from "../stores/viewer";
import {errorText, getJSON, postJSON} from "../lib/api";
import {fmtRate} from "../lib/fuzzy";
import type {Cls, LinkInfo, LiveResponse} from "../types";
import {Badge, Dot, Empty, ErrorBox, Sparkline, Toggle, type Tone} from "../components/ui";
import {FleetTotalsRow, LinkTotalsTable} from "../components/Bandwidth";
import type {Link} from "../lib/netstats";

const CLASSES: {cls: Cls; name: string; color: string}[] = [
  {cls: "C1", name: "state", color: "#35d07f"},
  {cls: "C2", name: "graph", color: "#4fc3f7"},
  {cls: "C3", name: "mesh", color: "#ffb020"},
  {cls: "C4", name: "camera stills", color: "#b388ff"},
  {cls: "C5", name: "video", color: "#ff6f91"},
  {cls: "C6", name: "image store", color: "#9ccc65"},
];

/** Known classes plus any other class the link reports a budget or rate for. */
export function linkClasses(l: LinkInfo): {cls: Cls; name: string; color: string}[] {
  const known = new Set(CLASSES.map(c => c.cls));
  const reported = new Set([...Object.keys(l.budgets ?? {}), ...Object.keys(l.rates ?? {})]);
  const extra = [...reported].filter(c => !known.has(c)).sort((a, b) => a.localeCompare(b, undefined, {numeric: true}));
  return [
    // C6 only when this robot's agent has it (older agents report C1-C5)
    ...CLASSES.filter(c => c.cls !== "C6" || reported.has("C6")),
    ...extra.map(cls => ({cls, name: cls, color: "#b0bec5"})),
  ];
}

// log-scale slider: 0..100 <-> 500 B/s .. 2 MB/s
const MIN = Math.log(500);
const MAX = Math.log(2_000_000);
export const sliderToBps = (v: number) => Math.round(Math.exp(MIN + ((MAX - MIN) * v) / 100));
export const bpsToSlider = (b: number) => Math.max(0, Math.min(100, ((Math.log(Math.max(b, 1)) - MIN) / (MAX - MIN)) * 100));

function staleTone(l: LinkInfo): Tone {
  if (!l.connected) return "err";
  const s = l.staleness_s ?? 0;
  return s > 10 ? "err" : s > 3 ? "warn" : "ok";
}

function BudgetRow({robot, cls, name, color, link, history}: {robot: string; cls: Cls; name: string; color: string; link: LinkInfo; history: number[]}) {
  const budget = link.budgets?.[cls] ?? 0;
  const [v, setV] = useState(bpsToSlider(budget));
  const [dragging, setDragging] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    if (!dragging) setV(bpsToSlider(budget));
  }, [budget, dragging]);
  const commit = async () => {
    setDragging(false);
    try {
      await postJSON(`api/live/${encodeURIComponent(robot)}/budget`, {cls, bps: sliderToBps(v)});
      setErr(null);
    } catch (e) {
      setErr(errorText(e));
    }
  };
  const rate = link.rates?.[cls] ?? 0;
  const util = budget ? rate / budget : 0;
  return (
    <div className="budget-row">
      <div className="budget-head">
        <span className="cls-tag" style={{borderColor: color, color}}>
          {cls}
        </span>
        <span className="small">{name}</span>
        <span className={`mono small ${util > 0.95 ? "tone-warn" : ""}`}>{fmtRate(rate)}</span>
      </div>
      <Sparkline values={history} width={150} height={26} color={color} max={budget || undefined} />
      <div className="budget-slider">
        <input
          type="range"
          min={0}
          max={100}
          step={1}
          value={v}
          aria-label={`${cls} budget`}
          onChange={e => {
            setDragging(true);
            setV(Number(e.target.value));
          }}
          onPointerUp={commit}
          onKeyUp={commit}
        />
        <span className="mono small budget-val">{fmtRate(sliderToBps(v))}</span>
      </div>
      {err && <div className="inline-err small">{err}</div>}
    </div>
  );
}

export function LivePanel() {
  const link = useRobots(s => s.link);
  const history = useRobots(s => s.rateHistory);
  const [err, setErr] = useState<unknown>(null);
  const [loaded, setLoaded] = useState(false);
  const {sources, enableSource} = useViewer();
  useEffect(() => {
    let alive = true;
    const poll = async () => {
      try {
        const r = await getJSON<LiveResponse>("api/live");
        if (!alive) return;
        for (const [name, l] of Object.entries(r.robots ?? {})) useRobots.getState().setLink(name, l);
        setErr(null);
      } catch (e) {
        if (alive) setErr(e);
      } finally {
        if (alive) setLoaded(true);
      }
    };
    void poll();
    const id = window.setInterval(poll, 2000);
    return () => {
      alive = false;
      window.clearInterval(id);
    };
  }, []);
  const robots = Object.entries(link);
  if (err && !robots.length) return <ErrorBox error={err} />;
  if (loaded && !robots.length) return <Empty title="No live robots">fleetlink has not heard from any robot. Is the console running with fleetlink enabled?</Empty>;
  return (
    <div className="live-panel">
      {robots.length > 0 && <FleetTotalsRow links={link as Record<string, Link>} />}
      {robots.map(([r, l]) => {
        const srcId = `live:${r}`;
        const src = sources[srcId];
        return (
          <section key={r} className="live-robot">
            <header className="row gap between">
              <span className="row gap">
                <Dot tone={staleTone(l)} />
                <strong>{r}</strong>
                <Badge tone={l.connected ? "ok" : "err"}>{l.connected ? "connected" : "disconnected"}</Badge>
              </span>
              {src && !src.enabled && (
                <button className="btn btn-xs" onClick={() => enableSource(srcId, true)}>
                  Show map
                </button>
              )}
            </header>
            <div className="live-stats">
              <div className="stat">
                <div className={`stat-v mono tone-${staleTone(l)}`}>{l.staleness_s !== null && l.staleness_s !== undefined ? `${l.staleness_s.toFixed(1)} s` : "-"}</div>
                <div className="stat-k">staleness</div>
              </div>
              <div className="stat">
                <div className="stat-v mono">{l.latency_ms !== null && l.latency_ms !== undefined ? `${l.latency_ms.toFixed(0)} ms` : "-"}</div>
                <div className="stat-k">latency</div>
              </div>
              <div className="stat">
                <div className="stat-v mono">{l.nodes ?? 0}</div>
                <div className="stat-k">nodes</div>
              </div>
              <div className="stat">
                <div className="stat-v mono">{l.tiles ?? 0}</div>
                <div className="stat-k">tiles</div>
              </div>
            </div>
            <div className="live-align small">
              <span className="muted">frame</span>
              <AlignStatus robot={r} />
              {!frameAlignOf(r) && <span className="muted">no alignment reported (older agent)</span>}
            </div>
            <div className="muted small mono">
              drops {l.drops ?? 0} · gaps {l.gaps ?? 0} · resyncs {l.resyncs ?? 0}
              {l.epoch ? ` · epoch ${String(l.epoch).slice(0, 8)}` : ""}
            </div>
            <LinkTotalsTable link={l as Link} />
            {linkClasses(l).map(c => (
              <BudgetRow key={c.cls} robot={r} cls={c.cls} name={c.name} color={c.color} link={l} history={history[r]?.[c.cls] ?? []} />
            ))}
            <div className="row gap between">
              <Toggle
                checked={Boolean(l.auto)}
                onChange={v => void postJSON(`api/live/${encodeURIComponent(r)}/auto`, {enabled: v}).catch(() => undefined)}
                label="Auto budget"
                title="Let the robot adapt budgets to measured link capacity"
              />
              <button className="btn btn-sm" onClick={() => void postJSON(`api/live/${encodeURIComponent(r)}/resync`).catch(() => undefined)}>
                Resync
              </button>
            </div>
          </section>
        );
      })}
    </div>
  );
}
