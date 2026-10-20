import {beforeEach, describe, expect, it} from "vitest";
import {compareWindow, fleetTotals, type IfaceStats, loadRadioChoices, type MachineIfaces, radioIface, RADIO_IFACE_KEY, sampleOf, saveRadioChoice} from "../src/lib/netstats";

const iface = (o: Partial<IfaceStats> = {}): IfaceStats => ({tx_rate_Bps: 1, rx_rate_Bps: 2, dt_s: 1, tx_total_B: 100, rx_total_B: 200, since_s: 10, addr: "", error: false, ...o});

describe("netstats", () => {
  beforeEach(() => window.localStorage.clear());

  it("sums fleet totals over links, per direction and per class", () => {
    const t = fleetTotals({
      a: {connected: true, bytes: {C1: 10, C2: 30}, rx_total_B: 40, tx_total_B: 5, rx_rate_Bps: 1, tx_rate_Bps: 2, rx_session_B: 20, tx_session_B: 1},
      b: {connected: false, bytes: {C1: 5}}, // receiver without direction totals: class sum
    });
    expect(t).toMatchObject({rx_total_B: 45, tx_total_B: 5, rx_rate_Bps: 1, tx_rate_Bps: 2, rx_session_B: 20, connected: 1, robots: 2});
    expect(t.bytes).toEqual({C1: 15, C2: 30});
  });

  it("compares over a common window and flags counter restarts", () => {
    const l = {connected: true, rx_total_B: 1000, tx_total_B: 100};
    const base = sampleOf(iface({tx_total_B: 5000, rx_total_B: 900}), l)!;
    const now = sampleOf(iface({tx_total_B: 8000, rx_total_B: 1400}), {...l, rx_total_B: 3000, tx_total_B: 300})!;
    expect(compareWindow(base, now)).toEqual({ifUp: 3000, ifDown: 500, flUp: 2000, flDown: 200, diffUp: 1000, diffDown: 300, ratio: 3500 / 2200});
    expect(compareWindow(now, base)).toBeNull();
    expect(sampleOf(iface({tx_total_B: null}), l)).toBeNull();
  });

  it("picks the operator's radio interface, else the topology default, and remembers it", () => {
    const m: MachineIfaces = {interfaces: {eth0: iface(), wlan0: iface()}, has_totals: true, radio_default: "eth0", radio_network: "silvus"};
    expect(radioIface(m)).toBe("eth0");
    expect(radioIface(m, "wlan0")).toBe("wlan0");
    expect(radioIface(m, "gone0")).toBe("eth0");
    expect(radioIface({...m, radio_default: null})).toBeNull();
    saveRadioChoice("hamilton", "wlan0");
    expect(loadRadioChoices()).toEqual({hamilton: "wlan0"});
    saveRadioChoice("hamilton", null);
    expect(loadRadioChoices()).toEqual({});
    window.localStorage.setItem(RADIO_IFACE_KEY, "{not json");
    expect(loadRadioChoices()).toEqual({});
  });
});
