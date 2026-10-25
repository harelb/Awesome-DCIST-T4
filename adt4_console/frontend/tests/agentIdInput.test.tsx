// @vitest-environment jsdom
import {afterEach, describe, expect, it} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {useMission} from "../src/stores/mission";
import {AgentIdInput} from "../src/viewer/MissionPanel";
import {useMapFull, exitMapFull} from "../src/viewer/mapFull";
import {useMapShortcuts} from "../src/viewer/shortcuts";

(globalThis as {IS_REACT_ACT_ENVIRONMENT?: boolean}).IS_REACT_ACT_ENVIRONMENT = true;
let root: Root | null = null;
afterEach(() => {
  act(() => root?.unmount());
  root = null;
  exitMapFull();
  document.body.innerHTML = "";
});

function Harness() {
  useMapShortcuts();
  const roster = useMission(s => s.roster);
  return (
    <ul>
      {roster.map(a => (
        <li key={a.id}>
          <AgentIdInput id={a.id} taken={roster.map(r => r.id)} />
        </li>
      ))}
    </ul>
  );
}

const type = (el: HTMLInputElement, ch: string) => {
  el.dispatchEvent(new KeyboardEvent("keydown", {key: ch, bubbles: true}));
  const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!;
  set.call(el, el.value + ch);
  el.dispatchEvent(new Event("input", {bubbles: true}));
};

describe("renaming an agent", () => {
  it("keeps the focus while typing (m does not toggle the full map) and renames on Enter", () => {
    const base = useMission.getState().roster[0] ?? {id: "spot1"};
    useMission.setState({roster: [{...(base as object), id: "spot1"} as never, {...(base as object), id: "husky1"} as never]});
    const host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<Harness />));
    const input = host.querySelector("input")!;
    input.focus();
    act(() => {
      for (const ch of "mm") type(input, ch);
    });
    expect(document.activeElement).toBe(input);
    expect(input.value).toBe("spot1mm");
    expect(useMapFull.getState().on).toBe(false);
    expect(useMission.getState().roster[0].id).toBe("spot1"); // not renamed while typing
    act(() => input.dispatchEvent(new KeyboardEvent("keydown", {key: "Enter", bubbles: true})));
    expect(useMission.getState().roster.map(a => a.id)).toEqual(["spot1mm", "husky1"]);
  });
  it("refuses an empty or duplicate id", () => {
    const base = useMission.getState().roster[0] ?? {id: "spot1"};
    useMission.setState({roster: [{...(base as object), id: "spot1"} as never, {...(base as object), id: "husky1"} as never]});
    const host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<Harness />));
    const input = host.querySelector("input")!;
    input.focus();
    act(() => {
      const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value")!.set!;
      set.call(input, "husky1");
      input.dispatchEvent(new Event("input", {bubbles: true}));
    });
    act(() => input.blur());
    expect(useMission.getState().roster.map(a => a.id)).toEqual(["spot1", "husky1"]);
    expect(input.value).toBe("spot1");
  });
});

import {agentEnds} from "../src/viewer/MissionOverlay";
import {parseStoredStyle} from "../src/stores/viewStyle";

describe("mission display", () => {
  it("an agent's plan ends at its last task's target (else its last path point)", () => {
    const tasks = {
      t1: {agent: "spot1", path: [[0, 0], [5, 0]], target: [5, 0]},
      t2: {agent: "husky1", path: [[1, 1], [2, 9]], target: null},
      t3: {agent: "spot1", path: [[5, 0], [9, 9]], target: [9, 9]},
    };
    expect(agentEnds({tasks} as never)).toEqual({spot1: [9, 9], husky1: [2, 9]});
    expect(agentEnds(null)).toEqual({});
  });
  it("opacity / marker settings are kept within range", () => {
    const s = parseStoredStyle(JSON.stringify({missionPathOpacity: 0.01, missionMarkerScale: 9, trajectoryOpacity: 0.4}));
    expect([s.missionPathOpacity, s.missionMarkerScale, s.trajectoryOpacity]).toEqual([0.15, 4, 0.4]);
  });
});
