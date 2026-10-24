import type {Tone} from "../components/ui";
import type {JobConfirm, PreflightCheck, PreflightStatus} from "../types";

/** Badge tone of a check status / machine verdict. */
export function preflightTone(status: PreflightStatus | string | undefined): Tone {
  return status === "go" ? "ok" : status === "warn" ? "warn" : status === "nogo" ? "err" : "off";
}

export function preflightLabel(status: PreflightStatus | string | undefined): string {
  return status === "nogo" ? "NO-GO" : status === "warn" ? "GO (warnings)" : status === "go" ? "GO" : String(status ?? "?").toUpperCase();
}

const RANK: Record<string, number> = {nogo: 0, warn: 1, info: 2, go: 3};

/** Checks grouped per machine (machines in `order` first), worst checks first. */
export function groupChecks(checks: PreflightCheck[], order: string[] = []): [string, PreflightCheck[]][] {
  const by = new Map<string, PreflightCheck[]>();
  for (const c of checks) by.set(c.machine, [...(by.get(c.machine) ?? []), c]);
  const names = [...order.filter(n => by.has(n)), ...[...by.keys()].filter(n => !order.includes(n)).sort()];
  return names.map(n => [n, [...by.get(n)!].sort((a, b) => (RANK[a.status] ?? 9) - (RANK[b.status] ?? 9) || a.name.localeCompare(b.name))]);
}

/** The NO-GO issues of a preflight confirmation (empty for other confirmations). */
export function preflightIssues(confirm: JobConfirm | null | undefined): PreflightCheck[] {
  if (!confirm || confirm.kind !== "preflight_nogo") return [];
  const issues = (confirm.detail as {issues?: unknown} | null | undefined)?.issues;
  return Array.isArray(issues) ? (issues as PreflightCheck[]) : [];
}

export function issueKey(c: PreflightCheck): string {
  return `${c.machine}\u0000${c.name}\u0000${c.detail}`;
}
