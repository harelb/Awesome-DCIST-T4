import {useEffect, useMemo, useState} from "react";
import {useSearchParams} from "react-router";
import {parseLaunchPreset} from "../lib/launchLink";
import {useFleet} from "../stores/fleet";
import {trackJob} from "../stores/jobs";
import {errorText, postJSON, useApi} from "../lib/api";
import type {ConsoleConfig, LaunchDefaults, LaunchPlan, MapEntry, MapsResponse} from "../types";
import {Badge, Card, Dot, Empty, ErrorBox, Field, FuzzyPicker, Loading, PageHeader, Segmented, Spinner, Toggle} from "../components/ui";
import {jobIdOf} from "./Fleet";

type Mode = "mapping" | "relocalize";

function defaultExperiment(mode: Mode) {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return `${mode === "mapping" ? "map" : "reloc"}_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
}

interface PriorOption {
  machine: string;
  map: MapEntry;
}

export function LaunchPage() {
  // Maps → "Launch with map" preselects the prior map (and relocalize) through the URL
  const [search] = useSearchParams();
  const preset = useMemo(() => parseLaunchPreset(search), [search]);
  const [mode, setMode] = useState<Mode>(preset?.mode ?? "mapping");
  const defaults = useApi<LaunchDefaults>(`api/launch/defaults?mode=${mode}`);
  const config = useApi<ConsoleConfig>("api/config");
  const maps = useApi<MapsResponse>("api/maps");
  const machines = useFleet(s => s.machines);

  const [robots, setRobots] = useState<string[]>([]);
  const [session, setSession] = useState("");
  const [baseStation, setBaseStation] = useState<string>("");
  const [bsSession, setBsSession] = useState("");
  const [experiment, setExperiment] = useState(defaultExperiment("mapping"));
  const [record, setRecord] = useState(false);
  const [prior, setPrior] = useState<string>(preset ? `${preset.source}\u0000${preset.prior}` : ""); // "machine\u0000path"
  const [dry, setDry] = useState<{lines: string[]} | null>(null);
  const [busy, setBusy] = useState<"dry" | "launch" | null>(null);
  const [err, setErr] = useState<string | null>(null);

  // apply presets whenever the mode's defaults arrive
  useEffect(() => {
    const d = defaults.data;
    if (!d) return;
    setRobots(d.robots ?? []);
    setSession(d.session ?? "");
    setBaseStation(d.base_station ?? "");
    setBsSession(d.bs_session ?? "");
    setDry(null);
  }, [defaults.data]);
  useEffect(() => setExperiment(defaultExperiment(mode)), [mode]);

  const sessions = defaults.data?.sessions ?? config.data?.sessions ?? [];
  const allMachines = machines?.machines ?? [];
  const robotMachines = allMachines.filter(m => m.role === "robot");
  const baseStations = allMachines.filter(m => m.role === "base_station");

  const priorOptions = useMemo<PriorOption[]>(() => {
    const out: PriorOption[] = [];
    for (const [machine, list] of Object.entries(maps.data?.machines ?? {})) {
      if (!Array.isArray(list)) continue;
      for (const m of list) if (m.path) out.push({machine, map: m});
    }
    return out.sort((a, b) => Number(Boolean(b.map.is_prior)) - Number(Boolean(a.map.is_prior)) || a.map.name.localeCompare(b.map.name));
  }, [maps.data]);

  const plan = (): LaunchPlan => {
    const [src, path] = prior ? prior.split("\u0000") : [null, ""];
    return {
      robots,
      session,
      experiment: experiment.trim(),
      mode,
      prior_path: path ?? "",
      prior_source: src,
      record,
      base_station: baseStation || null,
      bs_session: baseStation ? bsSession || null : null,
    };
  };

  const problems: string[] = [];
  if (!robots.length) problems.push("Select at least one robot");
  if (!session) problems.push("Pick a robot session");
  if (!experiment.trim()) problems.push("Name the experiment");
  else if (!/^[\w.-]+$/.test(experiment.trim())) problems.push("Experiment name: letters, digits, . _ - only");
  if (mode === "relocalize" && !prior) problems.push("Relocalize needs a prior map");

  const doDry = async () => {
    setBusy("dry");
    setErr(null);
    try {
      setDry(await postJSON<{lines: string[]}>("api/launch/dry_run", plan()));
    } catch (e) {
      setDry(null);
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };
  const doLaunch = async () => {
    setBusy("launch");
    setErr(null);
    try {
      const r = await postJSON("api/launch", plan());
      const id = jobIdOf(r);
      if (id) trackJob(id, `Launch ${experiment}`);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  const toggleRobot = (r: string) => setRobots(cur => (cur.includes(r) ? cur.filter(x => x !== r) : [...cur, r]));

  return (
    <div className="page">
      <PageHeader
        title="Launch"
        sub="Start the robot stack across the fleet, with an optional base station session and prior map."
        actions={
          <Segmented<Mode>
            ariaLabel="Launch preset"
            value={mode}
            onChange={setMode}
            options={[
              {value: "mapping", label: "Mapping"},
              {value: "relocalize", label: "Relocalize"},
            ]}
          />
        }
      />
      {defaults.error && <ErrorBox error={defaults.error} onRetry={defaults.reload} compact />}
      {defaults.loading && !defaults.data && <Loading label="Loading presets" />}
      <div className="launch-grid">
        <Card title="Robots">
          {robotMachines.length === 0 && <Empty title="No robots in topology" />}
          <div className="robot-pick">
            {robotMachines.map(m => (
              <label key={m.name} className={`pick ${robots.includes(m.name) ? "on" : ""} ${m.online ? "" : "offline"}`}>
                <input type="checkbox" checked={robots.includes(m.name)} onChange={() => toggleRobot(m.name)} />
                <Dot tone={m.online ? "ok" : "err"} />
                <span className="pick-name">{m.name}</span>
                <span className="muted small mono">{m.ip}</span>
                {!m.online && <Badge tone="err">offline</Badge>}
              </label>
            ))}
          </div>
          <Field label="Robot session">
            <FuzzyPicker ariaLabel="Robot session" value={session} options={sessions} onChange={setSession} placeholder="type to filter sessions" />
          </Field>
        </Card>

        <Card title="Base station">
          <Field label="Machine">
            <select className="input" value={baseStation} onChange={e => setBaseStation(e.target.value)} aria-label="Base station">
              <option value="">none (robots only)</option>
              {baseStations.map(m => (
                <option key={m.name} value={m.name}>
                  {m.name} {m.online ? "" : "(offline)"} {m.local ? "(this machine)" : ""}
                </option>
              ))}
            </select>
          </Field>
          <Field label="Base station session">
            <FuzzyPicker ariaLabel="Base station session" value={bsSession} options={sessions} onChange={setBsSession} placeholder="type to filter sessions" />
          </Field>
        </Card>

        <Card title="Experiment">
          <Field label="Experiment name" hint="Output goes to <output_root>/<name> on every machine.">
            <input className="input mono" value={experiment} onChange={e => setExperiment(e.target.value)} aria-label="Experiment name" />
          </Field>
          <Toggle checked={record} onChange={setRecord} label="Record rosbag" />
          <Field label={mode === "relocalize" ? "Prior map (required)" : "Prior map (optional)"}>
            {maps.error ? (
              <ErrorBox error={maps.error} compact onRetry={maps.reload} />
            ) : (
              <select className="input" value={prior} onChange={e => setPrior(e.target.value)} aria-label="Prior map">
                <option value="">none</option>
                {priorOptions.map(o => (
                  <option key={o.machine + o.map.path} value={`${o.machine}\u0000${o.map.path}`}>
                    {o.map.is_prior ? "[prior] " : ""}
                    {o.machine}: {o.map.name} ({String(o.map.size)})
                  </option>
                ))}
              </select>
            )}
          </Field>
        </Card>
      </div>

      <div className="launch-bar">
        <div className="launch-summary">
          {problems.length ? (
            <span className="tone-warn">{problems[0]}</span>
          ) : (
            <span>
              <strong>{mode}</strong> on <span className="mono">{robots.join(", ")}</span> with <span className="mono">{session}</span>
              {baseStation && (
                <>
                  {" "}
                  + <span className="mono">{baseStation}</span>
                </>
              )}
            </span>
          )}
        </div>
        <button className="btn" onClick={doDry} disabled={!!busy || problems.length > 0}>
          {busy === "dry" && <Spinner />} Dry run
        </button>
        <button className="btn btn-primary btn-lg" onClick={doLaunch} disabled={!!busy || problems.length > 0}>
          {busy === "launch" && <Spinner />} Launch
        </button>
      </div>
      {err && <ErrorBox error={err} compact />}
      {dry && (
        <Card title="Dry run: commands that would run" actions={<button className="btn btn-sm" onClick={() => setDry(null)}>Clear</button>}>
          <pre className="dryrun mono">{dry.lines.join("\n") || "(nothing to do)"}</pre>
        </Card>
      )}
    </div>
  );
}

