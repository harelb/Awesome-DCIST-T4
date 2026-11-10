import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {goalRobots, onlineRobots, planAutoSend, type Fleet} from "../src/lib/autoSend";
import {ChatPanel} from "../src/viewer/ChatPanel";
import {clearChat, useChat} from "../src/stores/chat";
import {AUTO_SEND_KEY, setAutoSend, useAutoSend} from "../src/stores/autoSend";
import {useRobots} from "../src/stores/robots";
import {useEngines} from "../src/stores/engines";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const NOW = 1_000_000;
const spotGuards = {estop_pressed: false, lease_known: true, lease_owned: true};

function fleet(): Fleet {
  return {
    state: {
      euclid: {platform: "spot", guards: {...spotGuards, lease_owned: false, lease_owner: "bosdyn.android.spotapp 8f80"}},
      hamilton: {platform: "spot", guards: {...spotGuards, estop_pressed: true}},
      gauss: {platform: "phoenix", guards: {estop_pressed: false, manual_mode: false}},
      lewis: {platform: "phoenix", guards: {estop_pressed: false}},
      apollo: {platform: "spot"},
    },
    stateAt: {euclid: NOW - 100, hamilton: NOW - 100, gauss: NOW - 100, lewis: NOW - 60_000, apollo: NOW - 100},
    link: {euclid: {connected: true}, hamilton: {connected: true}, gauss: {connected: true}, lewis: {connected: false}, apollo: {connected: true}},
  };
}

describe("auto-send rules", () => {
  it("only online robots with an executor and no E-stop; gauss is a husky; a held lease is taken, not skipped", () => {
    const {online, skipped} = onlineRobots(["euclid", "hamilton", "gauss", "lewis", "apollo"], fleet(), NOW);
    expect(online).toEqual([
      {name: "euclid", kind: "spot", leaseHolder: "bosdyn.android.spotapp 8f80"},
      {name: "gauss", kind: "husky"},
    ]);
    expect(Object.fromEntries(skipped.map(s => [s.name, s.reason]))).toEqual({
      apollo: "no executor status (runtime guards)",
      hamilton: "E-stopped",
      lewis: "offline",
    });
  });

  it("GRSTAPS-X leaves Huskies out", () => {
    const {online, skipped} = onlineRobots(["euclid", "gauss"], fleet(), NOW, "grstaps");
    expect(online.map(r => r.name)).toEqual(["euclid"]);
    expect(skipped).toContainEqual({name: "gauss", reason: "GRSTAPS-X leaves Huskies out"});
  });

  it("a goal naming a robot that is not online is not sent", () => {
    const {online, skipped} = onlineRobots(["euclid", "hamilton", "gauss"], fleet(), NOW);
    expect(goalRobots("(and (visited-poi-by gauss t1) (visited-object-by euclid o2))")).toEqual(["gauss", "euclid"]);
    expect(planAutoSend("(and (visited-poi-by gauss t1) (visited-object-by euclid o2))", online, skipped)).toMatchObject({via: "gauss", robots: ["gauss", "euclid"]});
    const p = planAutoSend("(visited-poi-by hamilton t1)", online, skipped);
    expect(p.via).toBeNull();
    expect(p.blocked).toContain("hamilton (E-stopped)");
  });
});

describe("chat auto-send", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  let calls: {url: string; body: Record<string, unknown> | null}[] = [];
  const GOAL = "(and (visited-poi-by gauss t8202) (visited-poi-by euclid t17657))";

  beforeEach(() => {
    calls = [];
    const now = Date.now();
    const f = fleet();
    useRobots.setState({state: f.state as never, stateAt: {euclid: now, gauss: now, hamilton: now}, link: f.link as never});
    useEngines.setState({engine: "fast_downward"} as never);
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url: string, init?: RequestInit) => {
        const body = init?.body ? JSON.parse(String(init.body)) : null;
        calls.push({url: String(url), body});
        if (String(url) === "api/chat")
          return new Response(JSON.stringify({type: "goal", kind: "pddl", body: {goal: GOAL}, robot: null}) + "\n", {status: 200});
        if (String(url).includes("/goal/pddl")) return new Response(JSON.stringify({ok: true, via: "goal_manager", lease: {euclid: {ok: true, message: "taken"}}}), {status: 200});
        return new Response("{}", {status: 200});
      }),
    );
  });
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.restoreAllMocks();
    clearChat();
    setAutoSend(false);
  });

  const mountAndAsk = async (text: string) => {
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    await act(async () => root!.render(<ChatPanel />));
    const ta = host.querySelector("textarea")!;
    await act(async () => {
      const set = Object.getOwnPropertyDescriptor(HTMLTextAreaElement.prototype, "value")!.set!;
      set.call(ta, text);
      ta.dispatchEvent(new Event("input", {bubbles: true}));
    });
    await act(async () => {
      ta.dispatchEvent(new KeyboardEvent("keydown", {key: "Enter", bubbles: true}));
    });
    await act(async () => {
      await new Promise(r => setTimeout(r, 20));
    });
  };

  it("off by default: a goal waits for the operator's Send", async () => {
    expect(useAutoSend.getState().on).toBe(false);
    await mountAndAsk("send gauss and euclid to the sidewalks");
    expect(calls.some(c => c.url === "api/chat")).toBe(true);
    expect(calls.some(c => c.url.includes("/goal/"))).toBe(false);
    expect(host!.querySelector("[data-testid=auto-send-on]")).toBeNull();
    expect(useChat.getState().items.some(i => i.type === "goal")).toBe(true);
  });

  it("on: plans for the online robots (gauss as a husky) and sends the PDDL goal once, routed by omniplanner", async () => {
    setAutoSend(true);
    expect(window.localStorage.getItem(AUTO_SEND_KEY)).toBe("1");
    await mountAndAsk("send gauss and euclid to the sidewalks");
    expect(host!.querySelector("[data-testid=auto-send-on]")).not.toBeNull();
    const chat = calls.find(c => c.url === "api/chat")!.body as {mission_ctx: {roster: {id: string; kind: string; binding: string; start: null}[]; auto_send: boolean}};
    expect(chat.mission_ctx.auto_send).toBe(true);
    expect(chat.mission_ctx.roster).toEqual([
      {id: "euclid", kind: "spot", start: null, binding: "euclid"},
      {id: "gauss", kind: "husky", start: null, binding: "gauss"},
    ]); // hamilton is E-stopped
    const sends = calls.filter(c => c.url.includes("/goal/"));
    expect(sends).toHaveLength(1);
    expect(sends[0].url).toBe("api/robots/gauss/goal/pddl");
    expect(sends[0].body).toEqual({goal: GOAL});
    expect(calls.some(c => c.url.includes("/goal/goto"))).toBe(false);
    const card = useChat.getState().items.find(i => i.type === "autosend");
    expect(card).toMatchObject({via: "gauss", robots: [{name: "gauss"}, {name: "euclid"}]});
    expect(host!.querySelector("[data-testid=auto-send-skipped]")!.textContent).toContain("hamilton (E-stopped)");
    expect(host!.querySelector("[data-testid=auto-send-stop-gauss]")!.textContent).toBe("Stop (manual)");
    expect(host!.querySelector("[data-testid=auto-send-stop-euclid]")!.textContent).toBe("Pause");
  });
});
