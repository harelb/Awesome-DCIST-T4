// Bandwidth totals: fleetlink link totals (/api/live) and per-machine interface
// counters from tx_rx_monitor (/api/network/interfaces), plus the radio-vs-fleetlink
// comparison. Pure helpers; the UI lives in viewer/LivePanel.tsx and pages/Network.tsx.

import type {LinkInfo} from "../types";

/** Fields fleetlink's Receiver.link() adds for byte totals (newer receivers only). */
export interface LinkTotals {
  bytes?: Record<string, number>; // robot -> base bytes per class since console start
  rx_total_B?: number; // robot -> base, all classes, since console start
  tx_total_B?: number; // base -> robot (control), since console start
  rx_rate_Bps?: number;
  tx_rate_Bps?: number;
  since_s?: number; // seconds since the receiver (console) started
  connected_since?: number | null; // epoch s the current link session began
  sessions?: number;
  rx_session_B?: number; // since the current session (connect / agent restart)
  tx_session_B?: number;
}
export type Link = LinkInfo & LinkTotals;

export interface IfaceStats {
  tx_rate_Bps: number | null;
  rx_rate_Bps: number | null;
  dt_s: number | null;
  tx_total_B: number | null; // since the monitor started (null: old monitor)
  rx_total_B: number | null;
  since_s: number | null;
  addr: string;
  error: boolean;
}
export interface MachineIfaces {
  status?: string;
  /** topology role: base_station | robot */
  role?: string | null;
  interfaces: Record<string, IfaceStats>;
  has_totals: boolean;
  radio_default: string | null;
  radio_network: string | null;
}
export interface InterfacesResponse {
  network: string;
  updated: number | null;
  machines: Record<string, MachineIfaces>;
}

export const RADIO_IFACE_KEY = "adt4.radioIface";

export const COMPARE_TOOLTIP =
  "The interface counts everything on the NIC: ssh, rsync, zenoh/TCP overhead, retransmits, " +
  "and in router mode all raw ROS traffic. fleetlink counts its own payload bytes after compression. " +
  "Robot TX on the radio interface is compared with fleetlink robot->base, robot RX with base->robot.";

export function loadRadioChoices(): Record<string, string> {
  try {
    const v = JSON.parse(window.localStorage.getItem(RADIO_IFACE_KEY) ?? "{}");
    return v && typeof v === "object" ? (v as Record<string, string>) : {};
  } catch {
    return {};
  }
}

export function saveRadioChoice(machine: string, iface: string | null): Record<string, string> {
  const cur = loadRadioChoices();
  if (iface) cur[machine] = iface;
  else delete cur[machine];
  try {
    window.localStorage.setItem(RADIO_IFACE_KEY, JSON.stringify(cur));
  } catch {
    /* private window / blocked storage: choice lasts for this page only */
  }
  return cur;
}

/** The operator's pick when that interface still exists, else the topology default. */
export function radioIface(m: MachineIfaces | undefined, chosen?: string): string | null {
  if (!m) return null;
  if (chosen && m.interfaces[chosen]) return chosen;
  return m.radio_default && m.interfaces[m.radio_default] ? m.radio_default : null;
}

export interface FleetTotals {
  rx_total_B: number;
  tx_total_B: number;
  rx_rate_Bps: number;
  tx_rate_Bps: number;
  rx_session_B: number;
  tx_session_B: number;
  connected: number;
  robots: number;
  bytes: Record<string, number>;
}

export function linkTotal(l: Link): number {
  return l.rx_total_B ?? Object.values(l.bytes ?? {}).reduce((a, b) => a + b, 0);
}

export function fleetTotals(links: Record<string, Link>): FleetTotals {
  const t: FleetTotals = {rx_total_B: 0, tx_total_B: 0, rx_rate_Bps: 0, tx_rate_Bps: 0, rx_session_B: 0, tx_session_B: 0, connected: 0, robots: 0, bytes: {}};
  for (const l of Object.values(links)) {
    t.robots++;
    if (l.connected) t.connected++;
    t.rx_total_B += linkTotal(l);
    t.tx_total_B += l.tx_total_B ?? 0;
    t.rx_rate_Bps += l.rx_rate_Bps ?? 0;
    t.tx_rate_Bps += l.tx_rate_Bps ?? 0;
    t.rx_session_B += l.rx_session_B ?? 0;
    t.tx_session_B += l.tx_session_B ?? 0;
    for (const [c, b] of Object.entries(l.bytes ?? {})) t.bytes[c] = (t.bytes[c] ?? 0) + b;
  }
  return t;
}

/** One sample of both counters for a robot (interface TX = up, RX = down). */
export interface CompareSample {
  ifUp: number;
  ifDown: number;
  flUp: number; // fleetlink robot -> base
  flDown: number; // fleetlink base -> robot
}

export interface Comparison {
  ifUp: number;
  ifDown: number;
  flUp: number;
  flDown: number;
  diffUp: number;
  diffDown: number;
  ratio: number | null; // interface / fleetlink, both directions
}

/**
 * Bytes since ``base`` on both counters. The cumulative totals start at different
 * times (monitor start vs console start), so the fair comparison is over a common
 * window. A counter that went backwards (monitor / console restart) returns null:
 * the caller re-baselines.
 */
export function compareWindow(base: CompareSample, now: CompareSample): Comparison | null {
  const d = {
    ifUp: now.ifUp - base.ifUp,
    ifDown: now.ifDown - base.ifDown,
    flUp: now.flUp - base.flUp,
    flDown: now.flDown - base.flDown,
  };
  if (d.ifUp < 0 || d.ifDown < 0 || d.flUp < 0 || d.flDown < 0) return null;
  const fl = d.flUp + d.flDown;
  return {...d, diffUp: d.ifUp - d.flUp, diffDown: d.ifDown - d.flDown, ratio: fl > 0 ? (d.ifUp + d.ifDown) / fl : null};
}

export function sampleOf(iface: IfaceStats | undefined, link: Link | undefined): CompareSample | null {
  if (!iface || !link || iface.tx_total_B === null || iface.rx_total_B === null) return null;
  return {ifUp: iface.tx_total_B, ifDown: iface.rx_total_B, flUp: linkTotal(link), flDown: link.tx_total_B ?? 0};
}

export function fmtDuration(s: number | null | undefined): string {
  if (s === null || s === undefined || !Number.isFinite(s)) return "-";
  if (s < 60) return `${Math.round(s)} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min ${Math.round(s % 60)} s`;
  return `${Math.floor(s / 3600)} h ${Math.round((s % 3600) / 60)} min`;
}

export interface RadioSide {
  machines: string[];
  rx_rate_Bps: number;
  tx_rate_Bps: number;
  rx_total_B: number;
  tx_total_B: number;
  /** shortest counter age among the summed interfaces (s) */
  since_s: number | null;
}

export interface RadioTotals {
  network: string | null;
  /** the base station's radio interface: what comes in from every robot */
  base: (RadioSide & {iface: string}) | null;
  /** the robots' radio interfaces, summed (their TX is what they put on the radio) */
  robots: RadioSide;
  /** machines with no radio interface to count (no address on the radio network) */
  missing: string[];
}

function side(): RadioSide {
  return {machines: [], rx_rate_Bps: 0, tx_rate_Bps: 0, rx_total_B: 0, tx_total_B: 0, since_s: null};
}

function add(t: RadioSide, machine: string, s: IfaceStats): void {
  t.machines.push(machine);
  t.rx_rate_Bps += s.rx_rate_Bps ?? 0;
  t.tx_rate_Bps += s.tx_rate_Bps ?? 0;
  t.rx_total_B += s.rx_total_B ?? 0;
  t.tx_total_B += s.tx_total_B ?? 0;
  if (s.since_s !== null && s.since_s !== undefined) t.since_s = t.since_s === null ? s.since_s : Math.min(t.since_s, s.since_s);
}

/** Radio traffic of the whole fleet from each machine's radio interface
 *  (``radioIface``: the operator's pick, else the address on the radio network). */
export function radioTotals(data: InterfacesResponse | undefined, choices: Record<string, string> = {}): RadioTotals {
  const out: RadioTotals = {network: null, base: null, robots: side(), missing: []};
  for (const [name, m] of Object.entries(data?.machines ?? {})) {
    const iface = radioIface(m, choices[name]);
    if (!iface) {
      out.missing.push(name);
      continue;
    }
    out.network = out.network ?? m.radio_network;
    const s = m.interfaces[iface];
    if (m.role === "base_station") {
      if (!out.base) out.base = {...side(), iface};
      add(out.base, name, s);
    } else add(out.robots, name, s);
  }
  return out;
}
