import {useEffect, useState, type ReactNode} from "react";
import {NavLink, Outlet, useLocation} from "react-router";
import {useWsStatus} from "../ws";
import {useFleet} from "../stores/fleet";
import {isActive, useJobs} from "../stores/jobs";
import {onBackendState} from "../lib/api";
import {JobConfirmHost, JobsDrawer} from "./Jobs";
import {Dot, Sheet} from "./ui";

const P = {
  fleet: "M3 5h18v4H3zM3 11h18v4H3zM3 17h18v4H3z M6 7h.01M6 13h.01M6 19h.01",
  launch: "M12 2l4 6h-3v8h-2V8H8zM5 18h14v3H5z",
  maps: "M3 6l6-3 6 3 6-3v15l-6 3-6-3-6 3zM9 3v15M15 6v15",
  viewer: "M12 2l9 5v10l-9 5-9-5V7zM12 12l9-5M12 12v10M12 12L3 7",
  network: "M12 20a2 2 0 100-4 2 2 0 000 4zM7.5 13.5a6.5 6.5 0 019 0M4.5 10.5a11 11 0 0115 0M1.5 7.5a15.5 15.5 0 0121 0",
  config: "M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0M14 4v4M8 10v4M16 16v4",
  jobs: "M4 5h16M4 10h16M4 15h10M4 20h7",
  layers: "M12 3l9 5-9 5-9-5zM3 13l9 5 9-5",
  search: "M11 18a7 7 0 100-14 7 7 0 000 14zM20.5 20.5l-4.6-4.6",
  panel: "M4 4h16v16H4zM4 13h16",
  more: "M3.5 12a1.5 1.5 0 103 0 1.5 1.5 0 10-3 0M10.5 12a1.5 1.5 0 103 0 1.5 1.5 0 10-3 0M17.5 12a1.5 1.5 0 103 0 1.5 1.5 0 10-3 0",
};

export function Icon({name, size = 18}: {name: keyof typeof P; size?: number}) {
  return (
    <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.7} strokeLinecap="round" strokeLinejoin="round" aria-hidden>
      <path d={P[name]} />
    </svg>
  );
}

const NAV: {to: string; label: string; short?: string; icon: keyof typeof P}[] = [
  {to: "/fleet", label: "Fleet", icon: "fleet"},
  {to: "/launch", label: "Launch", icon: "launch"},
  {to: "/maps", label: "Maps", icon: "maps"},
  {to: "/viewer", label: "Map viewer", short: "Viewer", icon: "viewer"},
  {to: "/network", label: "Network", icon: "network"},
  {to: "/config", label: "Config", icon: "config"},
];

function useBackendUp(): boolean {
  const [up, setUp] = useState(true);
  useEffect(() => onBackendState(ok => setUp(ok)), []);
  return up;
}

type PillTone = "ok" | "warn" | "err" | "off";
interface Health {
  key: string;
  tone: PillTone;
  label: ReactNode;
  detail: string;
}

function StatusPill({tone, label, title}: {tone: PillTone; label: ReactNode; title?: string}) {
  return (
    <span className="pill" title={title}>
      <Dot tone={tone} />
      {label}
    </span>
  );
}

function useHealth(): Health[] {
  const wsUp = useWsStatus(s => s.connected);
  const attempts = useWsStatus(s => s.attempts);
  const health = useFleet(s => s.health);
  const backendUp = useBackendUp();
  const neo = health ? Object.entries(health.neo4j ?? {}) : [];
  const stale = !backendUp; // last health is unknown while the server is unreachable
  return [
    {key: "server", tone: backendUp ? "ok" : "err", label: "server", detail: backendUp ? "Console server reachable" : "Console server unreachable"},
    {key: "live", tone: wsUp ? "ok" : attempts > 2 ? "err" : "warn", label: "live", detail: wsUp ? "WebSocket connected" : `WebSocket reconnecting (attempt ${attempts})`},
    {
      key: "ros",
      tone: !health || stale ? "off" : health.ros ? "ok" : "warn",
      label: "ROS",
      detail: !health || stale ? "Unknown" : health.ros ? "ROS bridge up" : `ROS not connected${health.ros_error ? `: ${health.ros_error}` : ""}`,
    },
    {
      key: "fleetlink",
      tone: !health || stale ? "off" : health.fleetlink ? "ok" : "warn",
      label: "fleetlink",
      detail: !health || stale ? "Unknown" : health.fleetlink ? "Live robot link up" : `fleetlink down${health.fleetlink_error ? `: ${health.fleetlink_error}` : ""}`,
    },
    ...neo.map(([n, ok]): Health => ({
      key: `neo4j:${n}`,
      tone: stale ? "off" : ok ? "ok" : "err",
      label: <span className="mono">neo4j:{n}</span>,
      detail: stale ? "Unknown" : ok ? "Graph database reachable" : "Graph database unreachable",
    })),
  ];
}

const RANK: Record<PillTone, number> = {ok: 0, off: 1, warn: 2, err: 3};

/** Phone top bar: one dot for everything; tap for the per-service list. */
function HealthButton({items}: {items: Health[]}) {
  const [open, setOpen] = useState(false);
  const worst = items.reduce<PillTone>((w, h) => (RANK[h.tone] > RANK[w] ? h.tone : w), "ok");
  const bad = items.filter(h => h.tone === "err" || h.tone === "warn").length;
  return (
    <>
      <button className="btn health-btn" onClick={() => setOpen(true)} aria-label="System status">
        <Dot tone={worst} />
        {bad ? `${bad} issue${bad > 1 ? "s" : ""}` : worst === "off" ? "status" : "all ok"}
      </button>
      {open && (
        <Sheet title="System status" onClose={() => setOpen(false)}>
          <ul className="health-list">
            {items.map(h => (
              <li key={h.key}>
                <Dot tone={h.tone} />
                <span className="strong">{h.label}</span>
                <span className="muted small">{h.detail}</span>
              </li>
            ))}
          </ul>
        </Sheet>
      )}
    </>
  );
}

function TopBar() {
  const items = useHealth();
  const jobs = useJobs(s => s.jobs);
  const open = useJobs(s => s.open);
  const drawerOpen = useJobs(s => s.drawerOpen);
  const close = useJobs(s => s.close);
  const active = Object.values(jobs).filter(isActive);
  const waiting = active.filter(j => j.state === "waiting").length;
  return (
    <div className="topbar">
      <div className="topbar-brand" aria-hidden>
        <Logo />
        ADT4
      </div>
      <div className="topbar-status">
        {items.map(h => (
          <StatusPill key={h.key} tone={h.tone} label={h.label} title={h.detail} />
        ))}
      </div>
      <HealthButton items={items} />
      <button className={`btn jobs-btn ${waiting ? "attention" : ""}`} onClick={() => (drawerOpen ? close() : open())} aria-label="Jobs">
        <Icon name="jobs" size={16} />
        Jobs
        {active.length > 0 && <span className="count">{active.length}</span>}
        {waiting > 0 && <span className="count count-warn">{waiting} waiting</span>}
      </button>
    </div>
  );
}

const Logo = ({size = 22}: {size?: number}) => (
  <svg width={size} height={size} viewBox="0 0 16 16" aria-hidden>
    <path d="M3 12L8 3l5 9z" fill="var(--ok)" />
  </svg>
);

/** Phone navigation: four main pages + "More" for the rest. */
function TabBar() {
  const [more, setMore] = useState(false);
  const {pathname} = useLocation();
  const main = NAV.slice(0, 4);
  const rest = NAV.slice(4);
  const inRest = rest.some(n => pathname.startsWith(n.to));
  return (
    <>
      <nav className="tabbar" aria-label="Main">
        {main.map(n => (
          <NavLink key={n.to} to={n.to} className={({isActive}) => `tab ${isActive ? "on" : ""}`}>
            <Icon name={n.icon} size={22} />
            <span>{n.short ?? n.label}</span>
          </NavLink>
        ))}
        <button className={`tab ${inRest ? "on" : ""}`} onClick={() => setMore(true)} aria-haspopup="dialog">
          <Icon name="more" size={22} />
          <span>More</span>
        </button>
      </nav>
      {more && (
        <Sheet title="More" onClose={() => setMore(false)}>
          <div className="sheet-links">
            {rest.map(n => (
              <NavLink key={n.to} to={n.to} className={({isActive}) => `sheet-link ${isActive ? "on" : ""}`} onClick={() => setMore(false)}>
                <Icon name={n.icon} size={20} />
                {n.label}
              </NavLink>
            ))}
          </div>
        </Sheet>
      )}
    </>
  );
}

export function Shell() {
  const backendUp = useBackendUp();
  const authError = useWsStatus(s => s.authError);
  return (
    <div className="shell">
      <nav className="sidenav" aria-label="Main">
        <div className="brand">
          <Logo />
          <span className="brand-text">
            ADT4 <span className="muted">console</span>
          </span>
        </div>
        {NAV.map(n => (
          <NavLink key={n.to} to={n.to} className={({isActive}) => `navlink ${isActive ? "on" : ""}`} title={n.label}>
            <Icon name={n.icon} />
            <span className="navlabel">{n.label}</span>
          </NavLink>
        ))}
      </nav>
      <div className="main">
        <TopBar />
        {!backendUp && (
          <div className="banner banner-err" role="alert">
            Console server unreachable. Showing the last known data; reconnecting automatically.
          </div>
        )}
        {backendUp && authError && (
          <div className="banner banner-err" role="alert">
            Not authorised: open the console with <span className="mono">?token=...</span> (the server has a token configured).
          </div>
        )}
        <div className="content">
          <Outlet />
        </div>
      </div>
      <TabBar />
      <JobsDrawer />
      <JobConfirmHost />
    </div>
  );
}
