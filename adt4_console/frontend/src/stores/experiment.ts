// Experiment runs (API.md §18): the active experiment, the open run, its bag
// recorders and the "all robots reached their goals" prompt.  The server
// publishes the whole status on `experiment/status` (latest value).
import {create} from "zustand";
import {ws} from "../ws";
import {getJSON, postJSON, sendJSON} from "../lib/api";

export type RecState = "starting" | "recording" | "stalled" | "unreachable" | "stopping" | "stopped" | "failed" | "disabled";
export type ChipTone = "ok" | "warn" | "err" | "off";

export interface MachineRec {
  machine: string;
  role: "robot" | "base_station" | string;
  platform?: string;
  host?: string;
  state: RecState;
  error: string | null;
  bag: string | null;
  segments: string[];
  elapsed_s: number | null;
  size_bytes: number;
  files: number;
  rate_bps: number | null;
  free_bytes: number | null;
  time_left_s: number | null;
  low_disk: boolean;
  probe_age_s: number | null;
  last_write_age_s?: number | null;
  topics: number;
}

export interface Outcome {
  label: "success" | "partial" | "fail" | "aborted";
  failure_category: string | null;
  notes: string;
}

export interface RunMetrics {
  mission_time_s: number | null;
  run_time_s: number | null;
  wall_time_s: number | null;
  planning_s: number | null;
  plans_sent: number;
  replans: number;
  plan_failures: number;
  interventions: number;
  interventions_by_type: Record<string, number>;
  intervention_s: number;
  total_path_m: number;
  robots_reached: number;
  llm_calls: number;
  llm_cost_usd: number;
  robots: Record<string, Record<string, unknown> & {path_m?: number | null; reached?: boolean; interventions?: number}>;
  outcome?: string | null;
  intervention_list?: Intervention[];
}

export interface Intervention {
  robot: string;
  type: string;
  start: number;
  end: number;
  open: boolean;
  duration_s: number;
  source: "auto" | "manual";
  note?: string | null;
  seq?: number | null;
}

export interface Run {
  id: string;
  experiment: string;
  created: number;
  status: "open" | "recording" | "ended" | "checked" | "offloaded";
  test_mode: boolean;
  record: boolean;
  profile: string;
  labels: {condition?: string; instruction_id?: string; tags?: string[]};
  instruction: string;
  robots: string[];
  t_start: number | null;
  t_plan_sent: number | null;
  t_all_reached: number | null;
  t_end: number | null;
  outcome: Outcome | null;
  metrics: RunMetrics | null;
  sanity: {t: number; level: "ok" | "warn" | "err" | "none"; machines: Record<string, string>} | null;
  offload: {t: number; dest: string; target: string; ok: boolean; deleted: string[]} | null;
  bag_bytes?: Record<string, number | null>;
}

export interface Experiment {
  id: string;
  name: string;
  notes: string;
  robots: string[];
  bag_profile: "full" | "light";
  auto_record: boolean;
  test_mode: boolean;
  created: number;
  phases: {mapping: Record<string, unknown> | null; editing: Record<string, unknown> | null};
  active_run: string | null;
  runs?: number;
  active?: boolean;
}

export interface Prompt {
  t: number;
  robots: Record<string, "reached" | "failed">;
  mission_time_s: number | null;
  paths?: Record<string, number | null>;
}

export interface ExperimentStatus {
  t: number;
  experiment: Experiment | null;
  run: Run | null;
  test_mode: boolean;
  recording: {enabled: boolean; summary: {tone: ChipTone; recording: number; total: number; size_bytes: number}; machines: MachineRec[]};
  logging: {events: number; poses: number; last_event_age_s: number | null; pose_age_s: Record<string, number>; ros_mirror: boolean | null; pose_stale?: string[]} | null;
  live: {wall_time_s: number | null; run_time_s: number | null} | null;
  track: Record<string, {state: string; plan_id: string | null; n: number | null}>;
  prompt: Prompt | null;
}

export interface ExperimentMeta {
  experiments: Experiment[];
  active: string | null;
  profiles: string[];
  outcomes: string[];
  failure_categories: string[];
  intervention_categories: string[];
}

interface ExperimentStore {
  status: ExperimentStatus | null;
  /** client time when `status` arrived (elapsed counters tick locally between frames) */
  rxAt: number;
  meta: ExperimentMeta | null;
  panelOpen: boolean;
  endOpen: boolean;
  /** prompts this browser dismissed (by prompt time) */
  dismissed: number | null;
  setStatus: (s: ExperimentStatus) => void;
  load: () => Promise<void>;
  loadMeta: () => Promise<void>;
  openPanel: (on: boolean) => void;
  openEnd: (on: boolean) => void;
  dismiss: (t: number) => void;
}

export const useExperiment = create<ExperimentStore>((set, get) => ({
  status: null,
  rxAt: 0,
  meta: null,
  panelOpen: false,
  endOpen: false,
  dismissed: null,
  setStatus: s => set({status: s, rxAt: Date.now()}),
  load: async () => {
    try {
      get().setStatus(await getJSON<ExperimentStatus>("api/experiments/status"));
    } catch {
      /* backend banner handles it */
    }
  },
  loadMeta: async () => {
    try {
      set({meta: await getJSON<ExperimentMeta>("api/experiments")});
    } catch {
      /* ignore */
    }
  },
  openPanel: on => set({panelOpen: on}),
  openEnd: on => set({endOpen: on}),
  dismiss: t => set({dismissed: t}),
}));

export function bindExperiment() {
  ws.on("experiment/status", d => {
    if (d && typeof d === "object") useExperiment.getState().setStatus(d as ExperimentStatus);
  });
  ws.onReconnect(() => void useExperiment.getState().load());
  void useExperiment.getState().load();
  void useExperiment.getState().loadMeta();
}

// ------------------------------------------------------------------ actions
const E = "api/experiments";
export const api = {
  create: (body: {name: string; robots: string[]; bag_profile: string; auto_record: boolean; notes?: string}) =>
    postJSON<Experiment>(E, body).then(r => (void useExperiment.getState().loadMeta(), r)),
  patch: (id: string, body: Partial<Experiment>) =>
    sendJSON<Experiment>("PATCH", `${E}/${encodeURIComponent(id)}`, body).then(r => (void useExperiment.getState().loadMeta(), r)),
  activate: (id: string | null) => postJSON<ExperimentStatus>(`${E}/activate`, {id}).then(r => (void useExperiment.getState().loadMeta(), r)),
  phase: (phase: "mapping" | "editing", note = "", source: string | null = null) => postJSON(`${E}/phase`, {phase, note, source}),
  openRun: (body: {labels?: Run["labels"]; instruction?: string; robots?: string[]; record?: boolean}) => postJSON<Run>(`${E}/run/open`, body),
  endRun: (stop_bags = true, outcome?: Outcome) => postJSON<{job_id: string}>(`${E}/run/end`, {stop_bags, outcome}),
  prompt: (action: "continue" | "dismiss") => postJSON<ExperimentStatus>(`${E}/run/prompt`, {action}),
  setRecord: (enabled: boolean) => postJSON<ExperimentStatus>(`${E}/run/record`, {enabled}),
  recorder: (machine: string, action: "restart" | "stop") => postJSON(`${E}/run/recorders/${encodeURIComponent(machine)}/${action}`),
  acceptMissing: (machines: string[]) => postJSON(`${E}/run/accept_missing`, {machines}),
  intervention: (robot: string | null, category: string, note: string) => postJSON(`${E}/run/intervention`, {robot, category, note}),
  retract: (seq: number, reason = "") => postJSON(`${E}/run/retract`, {seq, reason}),
  runs: (exp: string) => getJSON<{runs: Run[]}>(`${E}/${encodeURIComponent(exp)}/runs`),
  run: (exp: string, run: string) => getJSON<RunDetail>(`${E}/${encodeURIComponent(exp)}/runs/${encodeURIComponent(run)}`),
  outcome: (exp: string, run: string, o: Outcome) => postJSON<Run>(`${E}/${encodeURIComponent(exp)}/runs/${encodeURIComponent(run)}/outcome`, o),
  check: (exp: string, run: string) => postJSON<{job_id: string}>(`${E}/${encodeURIComponent(exp)}/runs/${encodeURIComponent(run)}/check`),
  offload: (exp: string, run: string, target: string, delete_after: boolean) =>
    postJSON<{job_id: string}>(`${E}/${encodeURIComponent(exp)}/runs/${encodeURIComponent(run)}/offload`, {target, delete_after}),
  exportUrl: (exp: string, table: "runs" | "robots", includeTest = false) =>
    `${E}/${encodeURIComponent(exp)}/export.csv?table=${table}${includeTest ? "&include_test=true" : ""}`,
};

export interface RunDetail {
  run: Run & {machines?: Record<string, MachineRec & {machine: unknown}>; plans?: {t: number; robot: string; kind: string; msg: unknown}[]};
  events: ({seq: number; t: number; kind: string; robot?: string} & Record<string, unknown>)[];
  metrics: RunMetrics | null;
  sanity: {level: string; machines: {machine: string; level: string; findings: {level: string; msg: string}[]; stats: Record<string, number | null>}[]} | null;
  recording: MachineRec[];
}

// ------------------------------------------------------------------ formatting
export function fmtBytes(n: number | null | undefined): string {
  if (n === null || n === undefined) return "—";
  if (n < 1024) return `${n} B`;
  const u = ["KB", "MB", "GB", "TB"];
  let v = n / 1024;
  let i = 0;
  while (v >= 1024 && i < u.length - 1) {
    v /= 1024;
    i++;
  }
  return `${v.toFixed(v < 10 ? 2 : v < 100 ? 1 : 0)} ${u[i]}`;
}

export function fmtDuration(s: number | null | undefined): string {
  if (s === null || s === undefined || !Number.isFinite(s)) return "—";
  const t = Math.max(0, Math.round(s));
  const h = Math.floor(t / 3600);
  const m = Math.floor((t % 3600) / 60);
  const sec = t % 60;
  return h ? `${h}:${String(m).padStart(2, "0")}:${String(sec).padStart(2, "0")}` : `${m}:${String(sec).padStart(2, "0")}`;
}

/** The header chip: what to show and how loud. */
export function chipView(st: ExperimentStatus | null): {show: boolean; tone: ChipTone; label: string; detail: string; pulse: boolean} {
  if (!st || !st.experiment) return {show: false, tone: "off", label: "", detail: "", pulse: false};
  const run = st.run;
  if (!run) {
    return {
      show: true,
      tone: st.test_mode ? "warn" : "off",
      label: st.test_mode ? "TEST · no run" : "no run",
      detail: `${st.experiment.name}: ${st.test_mode ? "testing mode (bags off)" : "a run opens (and bags start) when a plan is sent"}`,
      pulse: false,
    };
  }
  if (!st.recording.enabled) {
    return {show: true, tone: "warn", label: `NOT RECORDING · ${run.id.split("_")[0]}`, detail: "Testing mode: events are logged, bags are off", pulse: false};
  }
  const s = st.recording.summary;
  const label = `REC ${s.recording}/${s.total} · ${fmtBytes(s.size_bytes)}`;
  const bad = st.recording.machines.filter(m => m.state !== "recording");
  const detail = bad.length ? bad.map(m => `${m.machine}: ${m.state}${m.error ? ` (${m.error})` : ""}`).join("; ") : "All recorders writing";
  return {show: true, tone: s.tone, label, detail, pulse: s.tone === "err"};
}
