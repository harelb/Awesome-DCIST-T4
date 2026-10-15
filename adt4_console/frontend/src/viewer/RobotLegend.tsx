// Canvas legend: one chip per live robot (or robot with a plan) in its colour,
// so glyphs, plans and executor paths can be told apart at a glance.
import {useMemo} from "react";
import {useLiveRobots, useRobots} from "../stores/robots";
import {useRobotColorOf} from "../lib/palette";

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
        </span>
      ))}
    </div>
  );
}
