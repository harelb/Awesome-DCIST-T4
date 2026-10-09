// Browser ↔ console clock offset (Cristian's algorithm over GET /api/time).
// Lag figures combine times taken on three machines; the console already maps
// robot times into its clock (fleetlink ping offset), this maps the browser's.
import {getJSON} from "./api";

/** console clock minus browser clock, from the lowest-RTT sample. */
export function bestOffset(samples: {t0: number; t1: number; server: number}[]): number | null {
  let best: {rtt: number; off: number} | null = null;
  for (const s of samples) {
    const rtt = s.t1 - s.t0;
    const off = s.server - (s.t0 + s.t1) / 2;
    if (!best || rtt < best.rtt) best = {rtt, off};
  }
  return best ? best.off : null;
}

// Best (lowest-RTT) sample seen recently.  A sample's error is at most RTT/2,
// and samples taken while the page is busy (e.g. loading hundreds of mesh tiles)
// have inflated RTTs, so keep the best one and re-measure soon while it is poor.
let best: {rtt: number; off: number; at: number} | null = null;
let measuredAt = 0;
let inflight: Promise<void> | null = null;
const GOOD_RTT_S = 0.02;
const KEEP_BEST_MS = 5 * 60_000; // clocks drift; forget old samples eventually

async function measure(n = 5): Promise<void> {
  const samples = [];
  for (let i = 0; i < n; i++) {
    const t0 = Date.now() / 1000;
    const {t} = await getJSON<{t: number}>("api/time");
    samples.push({t0, t1: Date.now() / 1000, server: t});
  }
  for (const s of samples) {
    const rtt = s.t1 - s.t0;
    const fresh = best && Date.now() - best.at < KEEP_BEST_MS;
    if (!fresh || rtt < best!.rtt) best = {rtt, off: s.server - (s.t0 + s.t1) / 2, at: Date.now()};
  }
  measuredAt = Date.now();
}

/** Seconds to add to the browser clock to get console time (null until measured).
 * Re-measures every minute, or every 5 s while the best sample is still poor. */
export function consoleClockOffset(): number | null {
  const every = best && best.rtt <= GOOD_RTT_S ? 60_000 : 5_000;
  if (!inflight && Date.now() - measuredAt > every) {
    inflight = measure()
      .catch(() => undefined)
      .finally(() => {
        inflight = null;
      });
  }
  return best ? best.off : null;
}
