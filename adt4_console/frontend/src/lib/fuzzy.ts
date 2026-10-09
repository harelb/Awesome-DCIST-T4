/** Subsequence fuzzy score (lower is better), or null if `q` does not match `s`. */
export function fuzzyScore(q: string, s: string): number | null {
  q = q.toLowerCase();
  s = s.toLowerCase();
  if (!q) return 0;
  const sub = s.indexOf(q);
  if (sub >= 0) return sub;
  let score = 100;
  let pos = -1;
  for (const ch of q) {
    const nxt = s.indexOf(ch, pos + 1);
    if (nxt < 0) return null;
    score += nxt - pos - 1;
    pos = nxt;
  }
  return score;
}

export function fuzzyFilter<T>(items: T[], q: string, key: (t: T) => string): T[] {
  if (!q.trim()) return items;
  return items
    .map(it => [fuzzyScore(q.trim(), key(it)), it] as const)
    .filter((x): x is readonly [number, T] => x[0] !== null)
    .sort((a, b) => a[0] - b[0])
    .map(x => x[1]);
}

export function fmtBytes(n: number | null | undefined): string {
  if (n === null || n === undefined || !Number.isFinite(n)) return "-";
  const u = ["B", "kB", "MB", "GB", "TB"];
  let i = 0;
  while (Math.abs(n) >= 1000 && i < u.length - 1) {
    n /= 1000;
    i++;
  }
  return `${n.toFixed(n >= 100 || i === 0 ? 0 : 1)} ${u[i]}`;
}

export function fmtRate(bps: number | null | undefined): string {
  return bps === null || bps === undefined ? "-" : `${fmtBytes(bps)}/s`;
}

export function fmtAgo(epochS: number | null | undefined): string {
  if (!epochS) return "never";
  const d = Date.now() / 1000 - epochS;
  if (d < 2) return "just now";
  if (d < 60) return `${Math.round(d)} s ago`;
  if (d < 3600) return `${Math.round(d / 60)} min ago`;
  return `${Math.round(d / 3600)} h ago`;
}

export function fmtTime(epochS: number | null | undefined): string {
  if (!epochS) return "";
  return new Date(epochS * 1000).toLocaleTimeString([], {hour12: false});
}
