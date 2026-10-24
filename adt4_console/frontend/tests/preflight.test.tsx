import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {groupChecks, preflightIssues, preflightLabel, preflightTone} from "../src/lib/preflight";
import {ConfirmDialog} from "../src/components/Jobs";
import {PreflightPanel} from "../src/pages/Launch";
import type {Job, JobConfirm, PreflightCheck} from "../src/types";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const ouster: PreflightCheck = {machine: "gauss", name: "ouster", status: "nogo", detail: "Ouster does not answer ping at 192.168.92.99", hint: "power the Ouster"};
const spot: PreflightCheck = {machine: "hamilton", name: "spot_reach", status: "nogo", detail: "Spot does not answer ping at 10.0.0.3", hint: "power Spot on"};
const nogoConfirm: JobConfirm = {
  kind: "preflight_nogo",
  message: "Preflight: 2 NO-GO issue(s)",
  yes_label: "Launch anyway",
  no_label: "Abort",
  detail: {stage: "Preflight", issues: [ouster, spot]},
};

describe("preflight helpers", () => {
  it("maps statuses to tones and labels", () => {
    expect(preflightTone("go")).toBe("ok");
    expect(preflightTone("warn")).toBe("warn");
    expect(preflightTone("nogo")).toBe("err");
    expect(preflightLabel("nogo")).toBe("NO-GO");
    expect(preflightLabel("warn")).toBe("GO (warnings)");
  });
  it("groups per machine in the given order, worst first", () => {
    const checks: PreflightCheck[] = [
      {machine: "willow", name: "disk", status: "go", detail: ""},
      {machine: "gauss", name: "disk", status: "go", detail: ""},
      ouster,
      {machine: "gauss", name: "battery", status: "warn", detail: ""},
    ];
    const g = groupChecks(checks, ["gauss", "willow"]);
    expect(g.map(([m]) => m)).toEqual(["gauss", "willow"]);
    expect(g[0][1].map(c => c.name)).toEqual(["ouster", "battery", "disk"]);
  });
  it("only preflight confirmations carry issues", () => {
    expect(preflightIssues(nogoConfirm)).toHaveLength(2);
    expect(preflightIssues({kind: "kill_sessions", message: "x", detail: {issues: [ouster]}})).toEqual([]);
    expect(preflightIssues(null)).toEqual([]);
  });
});

describe("preflight UI", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  const mount = (el: React.ReactElement) => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(el));
  };
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.restoreAllMocks();
  });

  it("keeps Launch anyway disabled until every NO-GO issue is ticked", async () => {
    const fetchMock = vi.fn(async () => new Response("{}", {status: 200}));
    vi.stubGlobal("fetch", fetchMock);
    const job: Job = {id: "j1", kind: "launch", title: "Launch e1", state: "waiting", created: 0, confirm: nogoConfirm};
    const onDone = vi.fn();
    mount(<ConfirmDialog job={job} confirm={nogoConfirm} onDone={onDone} />);
    const yes = document.querySelector('[data-testid="confirm-yes"]') as HTMLButtonElement;
    expect(yes.textContent).toBe("Launch anyway");
    expect(document.body.textContent).toContain("gauss / ouster");
    expect(document.body.textContent).toContain("Fix: power Spot on");
    expect(yes.disabled).toBe(true);
    const boxes = [...document.querySelectorAll<HTMLInputElement>(".preflight-issue input")];
    expect(boxes).toHaveLength(2);
    act(() => boxes[0].click());
    expect(yes.disabled).toBe(true);
    act(() => boxes[1].click());
    expect(yes.disabled).toBe(false);
    act(() => boxes[1].click());
    expect(yes.disabled).toBe(true);
    act(() => boxes[1].click());
    await act(async () => yes.click());
    expect(fetchMock).toHaveBeenCalledWith(expect.stringContaining("api/jobs/j1/confirm"), expect.objectContaining({body: JSON.stringify({answer: true})}));
    expect(onDone).toHaveBeenCalled();
  });

  it("panel shows a verdict per machine with fix hints for problems", () => {
    mount(
      <PreflightPanel
        result={{
          checks: [ouster, {machine: "gauss", name: "zed", status: "go", detail: "ZED on USB", hint: "plug the ZED"}, {machine: "willow", name: "disk", status: "go", detail: "200 GB free"}],
          verdicts: {gauss: "nogo", willow: "go"},
          nogo: 1,
          t: 1700000000,
        }}
        order={["gauss", "willow"]}
        onClear={() => {}}
      />,
    );
    const gauss = document.querySelector('[data-testid="preflight-gauss"]')!;
    expect(gauss.textContent).toContain("NO-GO");
    expect(gauss.textContent).toContain("Fix: power the Ouster");
    expect(gauss.textContent).not.toContain("plug the ZED"); // hints only for problems
    expect(document.querySelector('[data-testid="preflight-willow"]')!.textContent).toContain("GO");
    expect(document.body.textContent).toContain("1 NO-GO");
  });
});
