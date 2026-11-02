// GRSTAPS-X's own schedule (API.md §15.5): when each robot starts and finishes each
// task as the solver planned it, a joint task drawn on every member's row. Used for
// a planned mission, a goal preview, and the live schedule a robot's planner publishes.
import {useMemo} from "react";

export interface ScheduleRow {
  robot: string;
  action: string;
  target: string;
  start: number;
  finish: number;
  coalition?: string[];
  /** a region goal's visit (API.md §15.5.1): its region and "explore r6 · 3/9" */
  region?: string;
  label?: string;
}
export interface PlannerScheduleData {
  makespan: number;
  rows: ScheduleRow[];
  precedence?: number[][];
  plan_id?: string;
  t?: number;
}

const ACTION_ICON: Record<string, string> = {
  "visit-location": "→",
  "inspect-object": "◎",
  "relocate-object": "⇄",
  "pick-object": "✋",
  "place-object": "⤓",
};

function fmt(s: number) {
  return s >= 90 ? `${(s / 60).toFixed(1)} min` : `${s.toFixed(0)} s`;
}

export function PlannerSchedule({
  schedule,
  colors,
  title = "GRSTAPS-X schedule",
  compact = false,
}: {
  schedule: PlannerScheduleData | null | undefined;
  colors?: Record<string, string>;
  title?: string;
  compact?: boolean;
}) {
  const robots = useMemo(() => [...new Set((schedule?.rows ?? []).map(r => r.robot))].sort(), [schedule]);
  if (!schedule || !schedule.rows.length) return null;
  const span = Math.max(schedule.makespan, ...schedule.rows.map(r => r.finish), 1);
  const pct = (s: number) => `${(100 * s) / span}%`;
  return (
    <div className={`planner-schedule${compact ? " compact" : ""}`} data-testid="planner-schedule">
      <div className="row gap small">
        <b>{title}</b>
        <span className="muted" title="The solver's makespan: when the last task finishes, at the nominal speeds and task durations it planned with">
          makespan {fmt(schedule.makespan)}
        </span>
      </div>
      <div className="mission-timeline">
        {robots.map(robot => (
          <div key={robot} className="tl-row">
            <span className="tl-agent" style={{color: colors?.[robot]}}>
              {robot}
            </span>
            <div className="tl-track">
              {schedule.rows
                .filter(r => r.robot === robot)
                .map((r, i) => {
                  const joint = (r.coalition?.length ?? 0) > 1;
                  return (
                    <div
                      key={`${r.action}-${r.target}-${i}`}
                      className={`tl-bar tl-planned${joint ? " tl-joint" : ""}`}
                      data-testid="schedule-bar"
                      style={{left: pct(r.start), width: `max(4px, ${pct(Math.max(0, r.finish - r.start))})`, background: colors?.[robot]}}
                      title={`${r.label ? `${r.label} at ${r.target}` : `${r.action} ${r.target}`} · ${fmt(r.start)}–${fmt(r.finish)}${joint ? ` · together with ${r.coalition!.filter(c => c !== robot).join(", ")}` : ""}`}
                    >
                      {r.label?.startsWith("explore") ? "⬚" : (ACTION_ICON[r.action] ?? "•")}
                      {joint ? "⛓" : ""}
                    </div>
                  );
                })}
            </div>
          </div>
        ))}
        <div className="tl-axis muted small">
          <span>0</span>
          <span>{fmt(span)}</span>
        </div>
      </div>
    </div>
  );
}
