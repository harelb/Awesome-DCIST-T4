// Canvas legend: one chip per live robot (or robot with a plan) in its colour,
// so glyphs, plans and executor paths can be told apart at a glance. Each live
// robot's chip also says whether its live layers are aligned to the global map
// frame (relocalized) or drawn in the robot's own odom frame (amber).
// The focused robot's chip is highlighted; clicking a chip focuses that robot
// (clicking the focused one goes back to all robots).
import {useMemo} from "react";
import {frameAlignOf, useLiveRobots, useRobots} from "../stores/robots";
import {useRobotColorOf} from "../lib/palette";
import {alignStatus} from "../lib/frameAlign";
import {useViewer} from "../stores/viewer";
import {useFocusedRobot} from "./useRobotFocus";

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
  const focused = useFocusedRobot();
  const setFocus = useViewer(s => s.setFocus);
  const robots = useMemo(() => [...new Set([...live, ...Object.keys(plans)])].sort(), [live, plans]);
  if (!robots.length) return null;
  return (
    <div className="robot-legend" aria-label="Robot colours">
      {robots.map(r => (
        <span
          key={r}
          role="button"
          tabIndex={0}
          aria-pressed={r === focused}
          className={`robot-chip ${r === focused ? "focused" : focused ? "dim" : ""}`}
          style={{borderColor: colorOf(r), ...(r === focused ? {boxShadow: `0 0 0 1px ${colorOf(r)}`} : {})}}
          title={r === focused ? "Focused · click for all robots" : `Focus ${r}`}
          onClick={() => setFocus(r === focused ? "all" : r)}
          onKeyDown={e => {
            if (e.key === "Enter" || e.key === " ") {
              e.preventDefault();
              setFocus(r === focused ? "all" : r);
            }
          }}
        >
          <span className="swatch" style={{background: colorOf(r)}} />
          {r}
          <AlignStatus robot={r} />
        </span>
      ))}
    </div>
  );
}
