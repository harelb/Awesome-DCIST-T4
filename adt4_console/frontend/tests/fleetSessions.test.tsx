import {afterEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {FleetPage} from "../src/pages/Fleet";
import {useFleet} from "../src/stores/fleet";
import {terminalUrl} from "../src/components/Terminal";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const json = (o: unknown, status = 200) => new Response(JSON.stringify(o), {status, headers: {"Content-Type": "application/json"}});
const flush = () => act(async () => void (await new Promise(r => setTimeout(r, 0))));

const MACHINES = {
  network: "mit_wifi",
  machines: [
    {name: "hamilton", role: "robot", ip: "10.29.236.1", user: "swarm", online: true, ping: 3, ssh: true},
    {name: "gauss", role: "robot", ip: "10.31.168.179", user: "swarm", online: false, ping: false, ssh: false},
  ],
};
const STATUS = {updated: Date.now() / 1000, machines: {hamilton: {tmux_sessions: "adt4_system, nstf_build"}}};

const buttons = (title: string) => [...document.querySelectorAll<HTMLButtonElement>("button")].filter(b => (b.title || b.textContent || "").includes(title));

describe("Fleet tmux sessions", () => {
  let root: Root | null = null;
  let host: HTMLDivElement | null = null;
  afterEach(() => {
    act(() => root?.unmount());
    host?.remove();
    vi.unstubAllGlobals();
  });
  const mount = (terminal: boolean) => {
    const fetchMock = vi.fn(async (url: string, init?: RequestInit) => {
      if (url === "api/config") return json({terminal});
      if (init?.method === "POST") return json({ok: true});
      return json({});
    });
    vi.stubGlobal("fetch", fetchMock);
    useFleet.setState({machines: MACHINES, status: STATUS as never, nodes: {robots: {}} as never, loaded: true, error: null, load: async () => {}});
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
    act(() => root!.render(<FleetPage />));
    return fetchMock;
  };

  it("kills a session after confirming, and warns about unsaved maps for adt4_system", async () => {
    const fetchMock = mount(false);
    await flush();
    expect(buttons("kill tmux session adt4_system")).toHaveLength(1);
    // terminals are off: no Terminal or attach buttons
    expect(buttons("Terminal")).toHaveLength(0);
    expect(buttons("attach to")).toHaveLength(0);

    act(() => buttons("kill tmux session adt4_system")[0].click());
    expect(document.body.textContent).toContain("tmux kill-session -t adt4_system");
    expect(document.body.textContent).toContain("Stop & Save first");
    expect(fetchMock.mock.calls.some(c => (c[1] as RequestInit | undefined)?.method === "POST")).toBe(false);

    await act(async () => buttons("Kill session")[0].click());
    await flush();
    const post = fetchMock.mock.calls.find(c => (c[1] as RequestInit | undefined)?.method === "POST")!;
    expect(post[0]).toBe("api/fleet/hamilton/tmux/adt4_system/kill");
  });

  it("stop sends Ctrl-C without the adt4_system warning for other sessions", async () => {
    const fetchMock = mount(false);
    await flush();
    act(() => buttons("Ctrl-C every pane of nstf_build")[0].click());
    expect(document.body.textContent).not.toContain("Stop & Save first");
    await act(async () => buttons("Send Ctrl-C")[0].click());
    await flush();
    expect(fetchMock.mock.calls.find(c => (c[1] as RequestInit | undefined)?.method === "POST")![0]).toBe("api/fleet/hamilton/tmux/nstf_build/stop");
  });

  it("offers terminals only on reachable machines when enabled", async () => {
    mount(true);
    await flush();
    // hamilton only: gauss is offline
    expect(buttons("shell on")).toHaveLength(1);
    expect(buttons("attach to")).toHaveLength(2);
  });

  it("builds the terminal websocket URL", () => {
    const u = new URL(terminalUrl("hamilton", "adt4_system", 120, 40));
    expect(u.protocol).toBe("ws:");
    expect(u.pathname.endsWith("/api/fleet/hamilton/terminal")).toBe(true);
    expect(Object.fromEntries(u.searchParams)).toMatchObject({cols: "120", rows: "40", session: "adt4_system"});
    expect(new URL(terminalUrl("hamilton", null, 80, 24)).searchParams.has("session")).toBe(false);
  });

  it("Stop & Save tears the stack down unless unchecked", async () => {
    const fetchMock = mount(false);
    await flush();
    const posts = () => fetchMock.mock.calls.filter(c => (c[1] as RequestInit | undefined)?.method === "POST");
    act(() => buttons("Stop & Save")[0].click());
    expect(document.body.textContent).toContain("shut down the rest of the stack");
    await act(async () => buttons("Stop & Save").at(-1)!.click());
    await flush();
    expect(posts()[0][0]).toBe("api/robots/hamilton/stop_save");
    expect(JSON.parse(String((posts()[0][1] as RequestInit).body))).toEqual({teardown: true});

    act(() => buttons("Stop & Save")[0].click());
    act(() => (document.querySelector(".modal input[type=checkbox]") as HTMLInputElement).click());
    await act(async () => buttons("Stop & Save").at(-1)!.click());
    await flush();
    expect(JSON.parse(String((posts()[1][1] as RequestInit).body))).toEqual({teardown: false});
  });
});
