// Silvus mesh at a glance: radios as nodes (named after the computer on their
// ethernet port), links coloured by quality with both directions' values, and a
// card per radio. The raw matrix / routing stay available under "Details".
import {useRobotColorOf} from "../lib/palette";
import {useIsPhone} from "../lib/media";
import {Badge} from "../components/ui";

export interface MeshRadio {
  name: string;
  node_id: number | null;
  battery: number | null;
  via: string | null;
  error: string | null;
  hosts: {ip: string; machine: string | null}[];
  local: boolean;
}

export interface MeshLink {
  a: string;
  b: string;
  ab: number | null;
  ba: number | null;
  unit: "dB" | "dBm";
  quality: "good" | "fair" | "poor" | "linked" | "unknown";
}

const QUALITY_COLOR: Record<MeshLink["quality"], string> = {
  good: "var(--ok)",
  fair: "var(--warn)",
  poor: "var(--err)",
  linked: "var(--muted)",
  unknown: "var(--line-2)",
};

// room around a node centre: the ring + antenna above, two text lines below
const TOP = 44;
const BOTTOM = 82;

/** Node centres for n radios (viewBox units): a row for 2 (a column when
 *  ``vertical``), else a ring that leaves room for the labels. */
export function meshLayout(n: number, w: number, h: number, vertical = false): [number, number][] {
  if (n === 1) return [[w / 2, (TOP + h - BOTTOM) / 2]];
  if (n === 2) return vertical ? [[w / 2, TOP], [w / 2, h - BOTTOM]] : [[w * 0.22, h / 2 - 16], [w * 0.78, h / 2 - 16]];
  const cy = (TOP + h - BOTTOM) / 2;
  const ry = (h - BOTTOM - TOP) / 2;
  const rx = w / 2 - 90;
  return Array.from({length: n}, (_, i) => {
    const t = -Math.PI / 2 + (2 * Math.PI * i) / n;
    return [w / 2 + rx * Math.cos(t), cy + ry * Math.sin(t)] as [number, number];
  });
}

/** viewBox size: wide on a desktop, a phone-shaped column on a phone. */
export function meshSize(n: number, phone: boolean): [number, number] {
  if (phone) return [360, n > 2 ? 440 : 330];
  return [640, n > 2 ? 360 : 210];
}

/** "51 / 50 dB" (a->b / b->a); one side only when the other radio did not answer. */
export function linkLabel(l: MeshLink): string {
  const f = (v: number | null) => (v === null || v === undefined ? "–" : Number(v).toFixed(0));
  if (l.ab === null && l.ba === null) return l.quality === "linked" ? "routed" : "?";
  return `${f(l.ab)} / ${f(l.ba)} ${l.unit}`;
}

/** The radio's display name: the computer behind it, else the radio name. */
export function radioTitle(r: MeshRadio): string {
  const named = r.hosts.map(h => h.machine).filter(Boolean);
  return named.length ? named.join(" + ") : r.name;
}

function Battery({pct}: {pct: number | null}) {
  if (pct === null || pct === undefined) return <span className="muted">–</span>;
  const tone = pct > 50 ? "ok" : pct > 20 ? "warn" : "err";
  return (
    <span className="mesh-battery" title={`${pct.toFixed(1)} %`}>
      <span className="mesh-battery-bar">
        <span className={`mesh-battery-fill tone-${tone}`} style={{width: `${Math.max(3, Math.min(100, pct))}%`}} />
      </span>
      <span className="mono small">{pct.toFixed(0)}%</span>
    </span>
  );
}

export function SilvusMesh({radios, links}: {radios: MeshRadio[]; links: MeshLink[]}) {
  const colorOf = useRobotColorOf();
  const phone = useIsPhone();
  const [W, H] = meshSize(radios.length, phone);
  const pos = meshLayout(radios.length, W, H, phone);
  const at = Object.fromEntries(radios.map((r, i) => [r.name, pos[i]]));
  const tint = (r: MeshRadio) => {
    if (r.error) return "var(--err)";
    const robot = r.hosts.find(h => h.machine)?.machine;
    return robot ? colorOf(robot) : "var(--accent)";
  };
  const titleOf = Object.fromEntries(radios.map(r => [r.name, radioTitle(r)]));

  return (
    <div className="mesh">
      <svg className="mesh-graph" viewBox={`0 0 ${W} ${H}`} role="img" aria-label="Silvus mesh: radios and links">
        {links.map(l => {
          const [x1, y1] = at[l.a];
          const [x2, y2] = at[l.b];
          const mx = (x1 + x2) / 2;
          const my = (y1 + y2) / 2;
          const color = QUALITY_COLOR[l.quality];
          const text = linkLabel(l);
          const tw = text.length * 7.4 + 18;
          return (
            <g key={l.a + l.b}>
              <title>{`${titleOf[l.a]} ↔ ${titleOf[l.b]}: ${l.quality} (${text}; ${titleOf[l.a]} hears ${titleOf[l.b]} / ${titleOf[l.b]} hears ${titleOf[l.a]})`}</title>
              <line x1={x1} y1={y1} x2={x2} y2={y2} stroke={color} strokeWidth={5} strokeOpacity={0.25} strokeLinecap="round" />
              <line
                x1={x1}
                y1={y1}
                x2={x2}
                y2={y2}
                stroke={color}
                strokeWidth={2.5}
                strokeLinecap="round"
                className={l.quality === "linked" || l.quality === "unknown" ? "mesh-link-dashed" : "mesh-link-live"}
              />
              <rect x={mx - tw / 2} y={my - 13} width={tw} height={26} rx={13} fill="var(--bg-2)" stroke={color} strokeWidth={1.2} />
              <text x={mx} y={my + 4.5} textAnchor="middle" className="mesh-link-text" fill="var(--text)">
                {text}
              </text>
            </g>
          );
        })}
        {radios.map((r, i) => {
          const [x, y] = pos[i];
          const c = tint(r);
          return (
            <g key={r.name} transform={`translate(${x} ${y})`}>
              <title>{`${r.name}${r.node_id ? ` · node ${r.node_id}` : ""}${r.error ? `\n${r.error}` : ""}`}</title>
              {r.local && <circle r={40} fill="none" stroke={c} strokeOpacity={0.35} strokeWidth={1.5} strokeDasharray="3 4" />}
              <circle r={31} fill="var(--bg-2)" stroke={c} strokeWidth={2.5} strokeDasharray={r.error ? "5 4" : undefined} />
              {/* antenna: mast + two arcs */}
              <g stroke={r.error ? "var(--err)" : "var(--text-2)"} strokeWidth={2} fill="none" strokeLinecap="round">
                <line x1={0} y1={-2} x2={0} y2={13} />
                <circle cx={0} cy={-5} r={2.6} fill={r.error ? "var(--err)" : "var(--text-2)"} stroke="none" />
                <path d="M -8 -13 A 11 11 0 0 0 -8 3" />
                <path d="M 8 -13 A 11 11 0 0 1 8 3" />
                <path d="M -13 -17 A 17 17 0 0 0 -13 7" strokeOpacity={0.6} />
                <path d="M 13 -17 A 17 17 0 0 1 13 7" strokeOpacity={0.6} />
              </g>
              <text y={54} textAnchor="middle" className="mesh-node-title" paintOrder="stroke" stroke="var(--bg-1)" strokeWidth={6} strokeLinejoin="round" fill={r.error ? "var(--err)" : c}>
                {titleOf[r.name]}
              </text>
              <text y={71} textAnchor="middle" className="mesh-node-sub" paintOrder="stroke" stroke="var(--bg-1)" strokeWidth={6} strokeLinejoin="round" fill="var(--muted)">
                {r.node_id ? `node ${r.node_id}` : r.name}
              </text>
            </g>
          );
        })}
      </svg>
      <div className="mesh-legend small muted">
        {(["good", "fair", "poor", "linked"] as const).map(q => (
          <span key={q} className="mesh-legend-item">
            <span className="mesh-swatch" style={{background: QUALITY_COLOR[q]}} /> {q === "linked" ? "routed, no reading" : q}
          </span>
        ))}
        <span className="mesh-legend-item">labels: A hears B / B hears A</span>
      </div>

      <div className="mesh-cards">
        {radios.map(r => (
          <div key={r.name} className={`mesh-card ${r.error ? "mesh-card-err" : ""}`} style={{borderTopColor: tint(r)}}>
            <div className="row gap between">
              <strong style={{color: r.error ? "var(--err)" : tint(r)}}>{radioTitle(r)}</strong>
              <span className="row gap-sm">
                {r.local && <Badge tone="info">this console</Badge>}
                {r.error ? <Badge tone="err">unreachable</Badge> : <Badge tone="ok">up</Badge>}
              </span>
            </div>
            <dl className="mesh-dl">
              <dt>Radio</dt>
              <dd className="mono">
                {r.name}
                {r.node_id ? <span className="muted"> · node {r.node_id}</span> : null}
              </dd>
              <dt>Battery</dt>
              <dd>
                <Battery pct={r.battery} />
              </dd>
              <dt>Computer</dt>
              <dd className="mono">{r.hosts.length ? r.hosts.map(h => `${h.ip}${h.machine ? ` (${h.machine})` : ""}`).join(", ") : <span className="muted">none seen</span>}</dd>
              <dt>Reached via</dt>
              <dd className="mono small ellipsis" title={r.via ?? ""}>
                {r.via ? (r.via.startsWith("fe80") ? `radio dongle (${r.via.split("%")[1] ?? "link-local"})` : r.via) : "–"}
              </dd>
              {links
                .filter(l => l.a === r.name || l.b === r.name)
                .map(l => {
                  const other = l.a === r.name ? l.b : l.a;
                  const hears = l.a === r.name ? l.ab : l.ba;
                  return [
                    <dt key={`t${other}`}>Hears {titleOf[other]}</dt>,
                    <dd key={`d${other}`}>
                      <span className="mesh-swatch" style={{background: QUALITY_COLOR[l.quality]}} /> {hears === null ? "–" : `${Number(hears).toFixed(0)} ${l.unit}`} <span className="muted">{l.quality}</span>
                    </dd>,
                  ];
                })}
            </dl>
            {r.error && <div className="mesh-err small mono">{r.error}</div>}
          </div>
        ))}
      </div>
    </div>
  );
}
