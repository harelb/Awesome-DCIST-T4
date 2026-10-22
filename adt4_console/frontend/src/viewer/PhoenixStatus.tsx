// Phoenix (Husky) status chips: AUTO / MANUAL, battery, E-STOP, SAFETY-STOP,
// from the fleetlink agent's guards. Nothing for other platforms.
import {phoenixChips} from "../lib/platform";
import {usePhoenixGuards, usePlatform} from "./usePlatform";

/** ``long``: MANUAL also says "will not execute plans" (Teleop / Plan panels). */
export function PhoenixChips({robot, long = false, className = ""}: {robot: string; long?: boolean; className?: string}) {
  const platform = usePlatform(robot);
  const g = usePhoenixGuards(robot);
  if (platform !== "phoenix") return null;
  const chips = phoenixChips(g);
  if (!chips.length) return null;
  return (
    <span className={`phx-chips ${className}`} data-testid={`phx-chips-${robot}`}>
      {chips.map(c => (
        <span key={c.key} className={`phx-chip tone-${c.tone} phx-${c.key}`} title={c.title}>
          {c.text}
          {long && c.key === "mode" && c.tone === "warn" ? " · will not execute plans" : ""}
        </span>
      ))}
    </span>
  );
}
