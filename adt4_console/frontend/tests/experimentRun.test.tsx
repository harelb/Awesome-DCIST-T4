import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {MemoryRouter} from "react-router";
import {postJSON, setRecordingBlockedHandler} from "../src/lib/api";
import {chipView, fmtBytes, fmtDuration, useExperiment, type ExperimentStatus, type MachineRec} from "../src/stores/experiment";
import {AllReachedPrompt, ExperimentBanners, RecChip, RecordingBlockedHost, RecordingPanel} from "../src/components/ExperimentRun";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const exp = {id: "e", name: "Lang2Mission", notes: "", robots: ["euclid", "hamilton"], bag_profile: "full" as const, auto_record: true,
  test_mode: false, created: 0, phases: {mapping: null, editing: null}, active_run: "r001"};

function mrec(machine: string, state: MachineRec["state"], extra: Partial<MachineRec> = {}): MachineRec {
  return {machine, role: "robot", state, error: null, bag: `b/${machine}`, segments: [machine], elapsed_s: 60, size_bytes: 2 * 1024 ** 3,
    files: 1, rate_bps: 5e6, free_bytes: 1e11, time_left_s: 20000, low_disk: false, probe_age_s: 1, last_write_age_s: 2, topics: 20, ...extra};
}

function status(over: Partial<ExperimentStatus> = {}, machines: MachineRec[] = [mrec("euclid", "recording"), mrec("hamilton", "recording")]): ExperimentStatus {
  const tone = machines.some(m => m.state === "failed") ? "err" : machines.some(m => m.state !== "recording") ? "warn" : "ok";
  return {
    t: 0, experiment: exp, test_mode: false,
    run: {id: "r001_20261102-120000", experiment: "e", created: 0, status: "recording", test_mode: false, record: true, profile: "full",
      labels: {}, instruction: "bring the cones", robots: ["euclid", "hamilton"], t_start: 0, t_plan_sent: 1, t_all_reached: null,
      t_end: null, outcome: null, metrics: null, sanity: null, offload: null},
    recording: {enabled: true, machines, summary: {tone, recording: machines.filter(m => m.state === "recording").length, total: machines.length,
      size_bytes: machines.reduce((a, m) => a + m.size_bytes, 0)}},
    logging: {events: 12, poses: 300, last_event_age_s: 1, pose_age_s: {euclid: 0.4}, ros_mirror: true, pose_stale: []},
    live: {wall_time_s: 65, run_time_s: 60}, track: {}, prompt: null,
    ...over,
  };
}

describe("experiment formatting + chip", () => {
  it("formats bytes and durations", () => {
    expect(fmtBytes(512)).toBe("512 B");
    expect(fmtBytes(3 * 1024 ** 3)).toBe("3.00 GB");
    expect(fmtDuration(3725)).toBe("1:02:05");
    expect(fmtDuration(65)).toBe("1:05");
    expect(fmtDuration(null)).toBe("—");
  });
  it("is hidden without an experiment and loud when a recorder failed", () => {
    expect(chipView(null).show).toBe(false);
    expect(chipView({...status(), experiment: null}).show).toBe(false);
    const ok = chipView(status());
    expect(ok.tone).toBe("ok");
    expect(ok.label).toMatch(/^REC 2\/2 · 4\.00 GB/);
    const bad = chipView(status({}, [mrec("euclid", "recording"), mrec("hamilton", "failed", {error: "recorder exited"})]));
    expect(bad.tone).toBe("err");
    expect(bad.pulse).toBe(true);
    expect(bad.detail).toContain("hamilton: failed (recorder exited)");
  });
  it("says NOT RECORDING in testing mode", () => {
    const st = status();
    st.run!.record = false;
    st.recording.enabled = false;
    expect(chipView(st).label).toContain("NOT RECORDING");
    expect(chipView({...status({run: null}), test_mode: true}).label).toContain("TEST");
  });
});

describe("experiment run UI", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  const mount = (el: React.ReactElement) => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<MemoryRouter>{el}</MemoryRouter>));
    return host;
  };
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    root = host = null;
    vi.unstubAllGlobals();
    useExperiment.setState({status: null, panelOpen: false, endOpen: false, dismissed: null});
  });

  it("shows the testing banner and the failed-recorder banner", () => {
    const st = status({}, [mrec("euclid", "failed", {error: "no ros2"})]);
    st.run!.record = true;
    useExperiment.getState().setStatus(st);
    const el = mount(<ExperimentBanners />);
    expect(el.querySelector("[data-testid=recorder-failed-banner]")?.textContent).toContain("euclid (no ros2)");
    expect(el.querySelector("[data-testid=testing-banner]")).toBeNull();
    act(() => useExperiment.getState().setStatus({...status({run: null}), test_mode: true, experiment: {...exp, test_mode: true}}));
    expect(el.querySelector("[data-testid=testing-banner]")?.textContent).toContain("NOT");
  });

  it("chip + panel list every machine with its state", () => {
    useExperiment.getState().setStatus(status({}, [mrec("euclid", "recording"), mrec("willow", "stalled", {role: "base_station", error: "bag has not grown for 70 s"})]));
    const el = mount(<><RecChip /><RecordingPanel embedded /></>);
    expect(el.querySelector("[data-testid=rec-chip]")?.className).toContain("rec-warn");
    expect(el.querySelector("[data-testid=rec-row-willow]")?.textContent).toContain("bag has not grown");
    expect(el.querySelector("[data-testid=rec-logging]")?.textContent).toContain("12 events");
  });

  it("asks end-or-continue when every robot reached its goal; continue keeps recording", async () => {
    const calls: string[] = [];
    vi.stubGlobal("fetch", vi.fn(async (url: string) => {
      calls.push(String(url));
      return new Response(JSON.stringify(status()), {status: 200});
    }));
    useExperiment.getState().setStatus(status({prompt: {t: 5, robots: {euclid: "reached", hamilton: "failed"}, mission_time_s: 125, paths: {euclid: 42.5, hamilton: null}}}));
    const el = mount(<AllReachedPrompt />);
    expect(el.ownerDocument.querySelector("[data-testid=all-reached]")?.textContent).toContain("2:05");
    expect(el.ownerDocument.body.textContent).toContain("42.5 m");
    await act(async () => (el.ownerDocument.querySelector("[data-testid=prompt-continue]") as HTMLButtonElement).click());
    expect(calls).toEqual(["api/experiments/run/prompt"]);
    act(() => (el.ownerDocument.querySelector("[data-testid=prompt-end]") as HTMLButtonElement).click());
    expect(useExperiment.getState().endOpen).toBe(true);
  });

  it("a goal refused for recording is re-sent after 'send without'", async () => {
    const sent: string[] = [];
    let goalTries = 0;
    vi.stubGlobal("fetch", vi.fn(async (url: string) => {
      sent.push(String(url));
      if (String(url).includes("/goal/")) {
        goalTries++;
        if (goalTries === 1) {
          return new Response(JSON.stringify({detail: "recording is not running on hamilton", recording_not_ready: true,
            machines: [{machine: "hamilton", state: "failed", error: "zsh: command not found: ros2"}]}), {status: 409});
        }
        return new Response(JSON.stringify({ok: true}), {status: 200});
      }
      return new Response("{}", {status: 200});
    }));
    const el = mount(<RecordingBlockedHost />);
    let result: unknown = null;
    let p!: Promise<unknown>;
    await act(async () => {
      p = postJSON("api/robots/euclid/goal/goto", {points: ["p1"]}).then(r => (result = r));
      await new Promise(r => setTimeout(r, 0));
    });
    const dlg = el.ownerDocument.querySelector("[data-testid=recording-blocked]");
    expect(dlg?.textContent).toContain("command not found: ros2");
    await act(async () => {
      (el.ownerDocument.querySelector("[data-testid=blocked-skip]") as HTMLButtonElement).click();
      await p;
    });
    expect(result).toEqual({ok: true});
    expect(sent).toEqual(["api/robots/euclid/goal/goto", "api/experiments/run/accept_missing", "api/robots/euclid/goal/goto"]);
    setRecordingBlockedHandler(null);
  });

  it("cancel lets the 409 reach the caller (goal not sent)", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => new Response(JSON.stringify({detail: "recording is not running on x", recording_not_ready: true, machines: []}), {status: 409})));
    const el = mount(<RecordingBlockedHost />);
    let err: unknown = null;
    let p!: Promise<unknown>;
    await act(async () => {
      p = postJSON("api/robots/x/goal/goto", {}).catch(e => (err = e));
      await new Promise(r => setTimeout(r, 0));
    });
    await act(async () => {
      (Array.from(el.ownerDocument.querySelectorAll("button")).find(b => b.textContent === "Cancel") as HTMLButtonElement).click();
      await p;
    });
    expect((err as {status: number}).status).toBe(409);
  });
});
