// Each robot's goals as images (API.md §15.5): the plan previews in the chat show,
// per robot, a cropped view of every object it is sent to (mask drawn). Clicking one
// opens it full size, with the object's other views and the mask switchable.
import {useEffect, useState} from "react";
import {getJSON} from "../lib/api";
import {withToken} from "../lib/auth";
import {Modal, Toggle} from "../components/ui";
import {srcPath, useViewer} from "../stores/viewer";
import {imageUrl} from "./objectImages";

interface NodeImages {
  images: string[];
  masks: boolean[];
  label: string;
}

const cache = new Map<string, Promise<NodeImages | null>>();

/** A node's images (node detail), cached per source + symbol; null: none / unknown. */
export function nodeImages(src: string, sym: string): Promise<NodeImages | null> {
  const key = `${src}|${sym}`;
  let p = cache.get(key);
  if (!p) {
    p = getJSON<{images?: string[]; masks?: boolean[]; label_name?: string; name?: string}>(`${srcPath(src)}/node/${encodeURIComponent(sym)}`)
      .then(d => (d.images?.length ? {images: d.images, masks: d.masks ?? [], label: d.name || d.label_name || ""} : null))
      .catch(() => null);
    cache.set(key, p);
  }
  return p;
}
export function _resetGoalImageCache() {
  cache.clear();
}

/** The graph source to show images from: the given one, else the Neo4j map, else the first shown. */
export function useImageSource(src?: string | null): string | null {
  const sources = useViewer(v => v.sources);
  const order = useViewer(v => v.order);
  if (src && sources[src]) return src;
  return order.find(id => sources[id]?.kind === "neo4j") ?? order.find(id => sources[id]?.enabled) ?? src ?? null;
}

function GoalThumb({src, sym, onOpen}: {src: string; sym: string; onOpen: (n: NodeImages) => void}) {
  const [n, setN] = useState<NodeImages | null | undefined>(undefined);
  useEffect(() => {
    let live = true;
    void nodeImages(src, sym).then(r => live && setN(r));
    return () => {
      live = false;
    };
  }, [src, sym]);
  if (n === undefined) return <span className="goal-thumb goal-thumb-empty mono small">{sym}</span>;
  if (n === null)
    return (
      <span className="goal-thumb goal-thumb-empty mono small" title={`${sym}: no image (a place, or an object never seen in a keyframe)`}>
        {sym}
      </span>
    );
  return (
    <button className="goal-thumb" onClick={() => onOpen(n)} title={`${sym}${n.label ? ` (${n.label})` : ""}: click to enlarge`} data-testid="goal-thumb">
      <img src={withToken(imageUrl(n.images[0], {w: 160, mask: Boolean(n.masks[0]), crop: true}))} alt={`${sym} ${n.label}`} loading="lazy" />
      <span className="goal-thumb-tag mono">{sym}</span>
    </button>
  );
}

/** Per robot: its goals as clickable images. ``goals``: {robot: [node symbol, ...]}. */
export function GoalImages({goals, src, colors}: {goals: Record<string, string[]> | null | undefined; src?: string | null; colors?: Record<string, string>}) {
  const s = useImageSource(src);
  const [open, setOpen] = useState<{sym: string; n: NodeImages; i: number} | null>(null);
  const [maskOn, setMaskOn] = useState(true);
  const rows = Object.entries(goals ?? {}).filter(([, syms]) => syms.length);
  if (!s || !rows.length) return null;
  const cur = open ? open.n.images[open.i] : null;
  const hasMask = open ? Boolean(open.n.masks[open.i]) : false;
  return (
    <div className="goal-images" data-testid="goal-images">
      {rows.map(([robot, syms]) => (
        <div key={robot} className="goal-images-row">
          <span className="goal-images-robot mono" style={{color: colors?.[robot]}}>
            {robot}
          </span>
          <div className="goal-images-strip">
            {syms.map(sym => (
              <GoalThumb key={sym} src={s} sym={sym} onOpen={n => setOpen({sym, n, i: 0})} />
            ))}
          </div>
        </div>
      ))}
      {open && cur && (
        <Modal title={`${open.sym}${open.n.label ? ` · ${open.n.label}` : ""}: view ${open.i + 1}/${open.n.images.length}`} onClose={() => setOpen(null)} wide>
          <img className="lightbox-img" src={withToken(imageUrl(cur, {mask: maskOn && hasMask}))} alt={`${open.sym} view ${open.i + 1}`} data-testid="goal-image-full" />
          <div className="row gap center">
            <button className="btn btn-sm" disabled={open.i === 0} onClick={() => setOpen({...open, i: open.i - 1})}>
              Previous
            </button>
            <Toggle checked={maskOn && hasMask} onChange={setMaskOn} label={hasMask ? "Mask" : "Mask (none for this view)"} />
            <button className="btn btn-sm" disabled={open.i >= open.n.images.length - 1} onClick={() => setOpen({...open, i: open.i + 1})}>
              Next
            </button>
          </div>
        </Modal>
      )}
    </div>
  );
}
