// "executing step 2/12 · 40 s" / "still trying step 1/3 · 3 min" / "plan done" /
// "plan not sent — the tablet holds the lease" / phoenix "executing (12 m to
// goal)" / "manual mode — not executing" for one robot (see planExec).
import {useEffect, useState} from "react";
import {useRobots} from "../stores/robots";
import {consoleClockOffset} from "../lib/clock";
import {planExecStatus} from "../lib/planExec";
import {useLease} from "./LeaseControl";
import {usePhoenixGuards, usePlatform} from "./usePlatform";

export function usePlanExec(robot: string) {
  const plan = useRobots(s => s.plan[robot]);
  const connected = useRobots(s => s.link[robot]?.connected);
  const teleop = useRobots(s => s.teleop[robot]?.state ?? null);
  const lease = useLease(robot);
  const platform = usePlatform(robot);
  const manual = usePhoenixGuards(robot).manual;
  const progress = useRobots(s => s.state[robot]?.phoenix_progress ?? null);
  // ages advance without new messages
  const [, setN] = useState(0);
  useEffect(() => {
    const h = setInterval(() => setN(x => x + 1), 2000);
    return () => clearInterval(h);
  }, []);
  return planExecStatus(robot, plan, {connected, teleop, lease, platform, manual, progress, now: Date.now() / 1000 + (consoleClockOffset() ?? 0)});
}

export function PlanExecStatus({robot, className = ""}: {robot: string; className?: string}) {
  const st = usePlanExec(robot);
  if (st.tone === "none") return null;
  return (
    <span className={`plan-exec tone-${st.tone} ${className}`} title={st.title} data-testid={`plan-exec-${robot}`} data-state={st.state}>
      {st.text}
    </span>
  );
}
