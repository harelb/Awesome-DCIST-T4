import {describe, expect, it} from "vitest";
import {radioTotals, type InterfacesResponse} from "../src/lib/netstats";

const ifc = (rx: number, tx: number, since = 600) => ({rx_rate_Bps: rx, tx_rate_Bps: tx, dt_s: 1, rx_total_B: rx * since, tx_total_B: tx * since, since_s: since, addr: "", error: false});
const data: InterfacesResponse = {
  network: "mit_wifi",
  updated: 0,
  machines: {
    willow: {role: "base_station", has_totals: true, radio_default: "enx3068", radio_network: "silvus", interfaces: {enx3068: ifc(24000, 800), wlp0s20f3: ifc(5000, 9000)}},
    euclid: {role: "robot", has_totals: true, radio_default: "enxf8e4", radio_network: "silvus", interfaces: {enxf8e4: ifc(400, 7200, 300)}},
    hamilton: {role: "robot", has_totals: true, radio_default: "enx2023", radio_network: "silvus", interfaces: {enx2023: ifc(400, 16500)}},
    gauss: {role: "robot", has_totals: true, radio_default: null, radio_network: null, interfaces: {wlan0: ifc(1, 1)}},
  },
};

describe("radio totals", () => {
  it("counts the base station's radio interface as everything coming in", () => {
    const t = radioTotals(data);
    expect(t.network).toBe("silvus");
    expect(t.base).toMatchObject({iface: "enx3068", machines: ["willow"], rx_rate_Bps: 24000, tx_rate_Bps: 800, rx_total_B: 24000 * 600});
  });
  it("sums the robots' radio interfaces and lists machines without one", () => {
    const t = radioTotals(data);
    expect(t.robots.machines).toEqual(["euclid", "hamilton"]);
    expect(t.robots.tx_rate_Bps).toBe(23700);
    expect(t.robots.since_s).toBe(300);
    expect(t.missing).toEqual(["gauss"]);
  });
  it("follows the operator's radio-interface pick", () => {
    expect(radioTotals(data, {willow: "wlp0s20f3"}).base).toMatchObject({iface: "wlp0s20f3", rx_rate_Bps: 5000});
  });
  it("no base station reporting: no base figure", () => {
    expect(radioTotals({...data, machines: {euclid: data.machines.euclid}}).base).toBeNull();
    expect(radioTotals(undefined).robots.machines).toEqual([]);
  });
});
