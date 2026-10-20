// Canvas legend: one chip per live robot (or robot with a plan) in its colour,
// so glyphs, plans and executor paths can be told apart at a glance. Each live
// robot's chip also says whether its live layers are aligned to the global map
// frame (relocalized) or drawn in the robot's own odom frame (amber).
import {useMemo} from "react";
import {frameAlignOf, useLiveRobots, useRobots} from "../stores/robots";
import {useRobotColorOf} from "../lib/palette";
import {alignStatus} from "../lib/frameAlign";

/** "aligned to map (relocalized) · 0.4 s" / "hamilton/odom frame — not relocalized". */
export function AlignStatus({robot, className = ""}: {robot: string; className?: string}) {
  const fa = useRobots(s => frameAlignOf(robot, s));
  const at = useRobots(s => s.stateAt[robot]);
  const st = alignStatus(fa, at ? (Date.now() - at) / 1000 : 0);
  if (st.tone === "none") return null;
  return (
    <span className={`align-status tone-${st.tone} ${className}`} title={st.title} data-testid={`align-${robot}`}>
      {st.text}
    </span>
  );
}

export function RobotLegend() {
  const live = useLiveRobots();
  const plans = useRobots(s => s.plan);
  const colorOf = useRobotColorOf();
  const robots = useMemo(() => [...new Set([...live, ...Object.keys(plans)])].sort(), [live, plans]);
  if (!robots.length) return null;
  return (
    <div className="robot-legend" aria-label="Robot colours">
      {robots.map(r => (
        <span key={r} className="robot-chip" style={{borderColor: colorOf(r)}}>
          <span className="swatch" style={{background: colorOf(r)}} />
          {r}
          <AlignStatus robot={r} />
        </span>
      ))}
    </div>
  );
}
