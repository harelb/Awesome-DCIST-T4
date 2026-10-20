// Toolbar control for the robot focus: "All robots" or one robot, with each
// robot's colour and liveness. Every robot-specific panel follows the choice.
// [ / ] cycle it (ViewerPage binds the keys).
import {useEffect, useRef, useState} from "react";
import {useViewer} from "../stores/viewer";
import {useRobotColorOf} from "../lib/palette";
import {useFocusedRobot, useRobotLiveness, useRobotRoster} from "./useRobotFocus";
import type {Focus, RobotLiveness} from "./robotFocus";

const LIVE_TEXT: Record<RobotLiveness, string> = {online: "online", stale: "stale", offline: "offline"};

export function RobotFocusPicker({compact = false}: {compact?: boolean}) {
  const roster = useRobotRoster();
  const live = useRobotLiveness(roster);
  const focused = useFocusedRobot();
  const setFocus = useViewer(s => s.setFocus);
  const colorOf = useRobotColorOf();
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState(0);
  const root = useRef<HTMLDivElement>(null);
  const options: Focus[] = ["all", ...roster];
  const cur = focused ?? "all";

  useEffect(() => {
    if (!open) return;
    setActive(Math.max(0, options.indexOf(cur)));
    const away = (e: PointerEvent) => !root.current?.contains(e.target as Node) && setOpen(false);
    window.addEventListener("pointerdown", away);
    return () => window.removeEventListener("pointerdown", away);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [open]);

  const choose = (f: Focus) => {
    setFocus(f);
    setOpen(false);
  };
  const online = roster.filter(r => live[r] === "online").length;
  const label = focused ? focused : compact ? "All" : "All robots";
  return (
    <div className="focus-picker" ref={root}>
      <button
        className={`btn btn-sm focus-btn ${focused ? "focused" : ""}`}
        style={focused ? {borderColor: colorOf(focused)} : undefined}
        aria-haspopup="listbox"
        aria-expanded={open}
        aria-label={`Robot focus: ${focused ?? "all robots"}`}
        title="Robot focus: camera, teleop, plan, live link and the map follow it · [ / ] cycle"
        data-testid="focus-picker"
        onClick={() => setOpen(o => !o)}
        onKeyDown={e => {
          if (!open && (e.key === "ArrowDown" || e.key === "ArrowUp")) {
            e.preventDefault();
            setOpen(true);
          } else if (open) {
            if (e.key === "ArrowDown") setActive(a => (a + 1) % options.length);
            else if (e.key === "ArrowUp") setActive(a => (a - 1 + options.length) % options.length);
            else if (e.key === "Enter" || e.key === " ") choose(options[active]);
            else if (e.key === "Escape") setOpen(false);
            else return;
            e.preventDefault();
            e.stopPropagation();
          }
        }}
      >
        {focused ? <span className={`focus-dot live-${live[focused] ?? "offline"}`} style={{background: colorOf(focused)}} /> : <span className="focus-all-mark" aria-hidden>◎</span>}
        <span className="focus-name">{label}</span>
        {!compact && (focused ? <span className={`focus-live live-${live[focused] ?? "offline"}`}>{LIVE_TEXT[live[focused] ?? "offline"]}</span> : roster.length > 0 && <span className="focus-live">{online}/{roster.length} online</span>)}
        <span className="focus-caret" aria-hidden>
          ▾
        </span>
      </button>
      {open && (
        <ul className="focus-menu" role="listbox" aria-label="Robot focus">
          {options.map((o, i) => (
            <li
              key={o}
              role="option"
              aria-selected={o === cur}
              className={`${o === cur ? "selected" : ""} ${i === active ? "active" : ""}`}
              onPointerEnter={() => setActive(i)}
              onClick={() => choose(o)}
            >
              {o === "all" ? (
                <>
                  <span className="focus-all-mark" aria-hidden>
                    ◎
                  </span>
                  <span className="focus-name">All robots</span>
                  <span className="focus-live muted">
                    {online}/{roster.length} online
                  </span>
                </>
              ) : (
                <>
                  <span className={`focus-dot live-${live[o]}`} style={{background: colorOf(o)}} />
                  <span className="focus-name">{o}</span>
                  <span className={`focus-live live-${live[o]}`}>{LIVE_TEXT[live[o]]}</span>
                </>
              )}
            </li>
          ))}
          {!roster.length && <li className="muted small focus-empty">No robots known yet</li>}
        </ul>
      )}
    </div>
  );
}

/** A small "robot" chip shown in a panel when the panel follows the focus. */
export function FocusChip({robot}: {robot: string}) {
  const colorOf = useRobotColorOf();
  const setFocus = useViewer(s => s.setFocus);
  return (
    <span className="focus-chip" style={{borderColor: colorOf(robot)}} title="This panel follows the robot focus (toolbar)">
      <span className="focus-dot" style={{background: colorOf(robot)}} />
      {robot}
      <button className="focus-chip-x" onClick={() => setFocus("all")} aria-label="Show all robots" title="Back to all robots">
        ×
      </button>
    </span>
  );
}
