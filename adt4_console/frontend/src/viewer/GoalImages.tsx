// Each robot's goals as images (API.md §15.5), in the chat's plan previews and the
// mission tab. An object shows its own views (cropped, mask drawn); a region or place
// goal shows the keyframe / sub-keyframe taken nearest to where the plan ends there,
// facing the way the robot arrives. Clicking one opens it full size, with the other
// views (Previous / Next) and, for an object, the mask switchable.
import {useEffect, useState} from "react";
import {getJSON} from "../lib/api";
import {withToken} from "../lib/auth";
import {Modal, Toggle} from "../components/ui";
import {srcPath, useViewer} from "../stores/viewer";
import {imageUrl} from "./objectImages";

/** A goal: a node symbol, with where the plan ends there (map x, y; heading rad). */
export interface GoalPoint {
  sym: string;
  xy?: number[] | null;
  yaw?: number | null;
}
export type GoalsByRobot = Record<string, (GoalPoint | string)[]>;

interface View {
  url: string;
  mask: boolean;
  note: string;
}
interface GoalViews {
  views: View[];
  label: string;
  /** "object": the node's own images; "near": frames nearest to the plan's end there */
  kind: "object" | "near";
}

const cache = new Map<string, Promise<GoalViews | null>>();

function fmtM(m: number | null | undefined) {
  return m == null ? "" : `${m < 10 ? m.toFixed(1) : Math.round(m)} m`;
}

/** A goal's views, cached: the object's images, else the frames nearest its end point; null: none. */
export function goalViews(src: string, g: GoalPoint): Promise<GoalViews | null> {
  const key = `${src}|${g.sym}|${g.xy?.map(v => v.toFixed(1)).join(",") ?? ""}|${g.yaw?.toFixed(2) ?? ""}`;
  let p = cache.get(key);
  if (!p) {
    p = (async () => {
      try {
        const d = await getJSON<{images?: string[]; masks?: boolean[]; label_name?: string; name?: string}>(`${srcPath(src)}/node/${encodeURIComponent(g.sym)}`);
        const imgs = d.images ?? [];
        if (imgs.length) {
          return {views: imgs.map((u, i) => ({url: u, mask: Boolean(d.masks?.[i]), note: `view ${i + 1}/${imgs.length}`})), label: d.name || d.label_name || "", kind: "object" as const};
        }
      } catch {
        /* an unknown symbol: try the nearest frames */
      }
      if (!g.xy || g.xy.length < 2) return null;
      try {
        const q = new URLSearchParams({x: String(g.xy[0]), y: String(g.xy[1]), n: "6"});
        if (g.yaw != null) q.set("yaw", String(g.yaw));
        const r = await getJSON<{frames: {image: string; kind: string; dist_m: number | null; yaw_deg: number | null}[]}>(`${srcPath(src)}/near?${q}`);
        if (!r.frames.length) return null;
        return {
          views: r.frames.map(f => ({
            url: f.image,
            mask: false,
            note: `${f.kind === "keyframe" ? "keyframe" : "sub-keyframe"} ${fmtM(f.dist_m)} from where the plan ends${f.yaw_deg != null ? `, ${Math.round(f.yaw_deg)}° off its heading` : ""}`,
          })),
          label: "nearest view",
          kind: "near" as const,
        };
      } catch {
        return null;
      }
    })();
    cache.set(key, p);
  }
  return p;
}
export const _resetGoalImageCache = () => cache.clear();

/** The graph source to show images from: the given one, else the Neo4j map, else the first shown. */
export function useImageSource(src?: string | null): string | null {
  const sources = useViewer(v => v.sources);
  const order = useViewer(v => v.order);
  if (src && sources[src]) return src;
  return order.find(id => sources[id]?.kind === "neo4j") ?? order.find(id => sources[id]?.enabled) ?? src ?? null;
}

const asPoint = (g: GoalPoint | string): GoalPoint => (typeof g === "string" ? {sym: g} : g);

function GoalThumb({src, goal, onOpen}: {src: string; goal: GoalPoint; onOpen: (v: GoalViews) => void}) {
  const [v, setV] = useState<GoalViews | null | undefined>(undefined);
  const key = `${goal.sym}|${goal.xy?.join(",") ?? ""}|${goal.yaw ?? ""}`;
  useEffect(() => {
    let live = true;
    void goalViews(src, goal).then(r => live && setV(r));
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [src, key]);
  if (!v)
    return (
      <span className="goal-thumb goal-thumb-empty mono small" title={v === null ? `${goal.sym}: no image near it` : goal.sym}>
        {goal.sym}
      </span>
    );
  const first = v.views[0];
  const thumb = v.kind === "object" ? imageUrl(first.url, {w: 160, mask: first.mask, crop: true}) : imageUrl(first.url, {w: 160});
  return (
    <button className="goal-thumb" onClick={() => onOpen(v)} title={`${goal.sym}${v.label ? ` (${v.label})` : ""}${v.kind === "near" ? `: ${first.note}` : ""}: click to enlarge`} data-testid="goal-thumb">
      <img src={withToken(thumb)} alt={`${goal.sym} ${v.label}`} loading="lazy" />
      <span className="goal-thumb-tag mono">
        {goal.sym}
        {v.kind === "near" ? " ⌖" : ""}
      </span>
    </button>
  );
}

/** Per robot: its goals as clickable images. */
export function GoalImages({goals, src, colors}: {goals: GoalsByRobot | null | undefined; src?: string | null; colors?: Record<string, string>}) {
  const s = useImageSource(src);
  const [open, setOpen] = useState<{sym: string; v: GoalViews; i: number} | null>(null);
  const [maskOn, setMaskOn] = useState(true);
  const rows = Object.entries(goals ?? {}).filter(([, gs]) => gs.length);
  if (!s || !rows.length) return null;
  const cur = open ? open.v.views[open.i] : null;
  return (
    <div className="goal-images" data-testid="goal-images">
      {rows.map(([robot, gs]) => (
        <div key={robot} className="goal-images-row">
          <span className="goal-images-robot mono" style={{color: colors?.[robot]}}>
            {robot}
          </span>
          <div className="goal-images-strip">
            {gs.map(asPoint).map(g => (
              <GoalThumb key={g.sym} src={s} goal={g} onOpen={v => setOpen({sym: g.sym, v, i: 0})} />
            ))}
          </div>
        </div>
      ))}
      {open && cur && (
        <Modal title={`${open.sym}${open.v.label ? ` · ${open.v.label}` : ""}: ${cur.note}`} onClose={() => setOpen(null)} wide>
          <img className="lightbox-img" src={withToken(imageUrl(cur.url, {mask: maskOn && cur.mask}))} alt={`${open.sym} ${cur.note}`} data-testid="goal-image-full" />
          <div className="row gap center">
            <button className="btn btn-sm" disabled={open.i === 0} onClick={() => setOpen({...open, i: open.i - 1})}>
              Previous
            </button>
            {open.v.kind === "object" ? (
              <Toggle checked={maskOn && cur.mask} onChange={setMaskOn} label={cur.mask ? "Mask" : "Mask (none for this view)"} />
            ) : (
              <span className="muted small">{open.i === 0 ? "nearest view to where the plan ends" : `${open.i + 1}. nearest view`}</span>
            )}
            <button className="btn btn-sm" disabled={open.i >= open.v.views.length - 1} onClick={() => setOpen({...open, i: open.i + 1})}>
              Next
            </button>
          </div>
        </Modal>
      )}
    </div>
  );
}
