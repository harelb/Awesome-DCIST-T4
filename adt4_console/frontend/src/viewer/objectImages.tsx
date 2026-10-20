// Object images from the image store: search-list thumbnails (lazy, server-side
// resized/cropped with the SAM3 mask overlay) and the Inspect hero image.
import {useEffect, useRef, useState, type RefObject} from "react";
import {getJSON} from "../lib/api";
import {withToken} from "../lib/auth";
import {Modal, Toggle} from "../components/ui";
import {srcPath} from "../stores/viewer";
import "./objectImages.css";

export interface ImageOpts {
  w?: number;
  mask?: boolean;
  crop?: boolean;
}

/** URL of one node image variant (w= resize, mask=1 overlay, crop=1 crop to object). */
export function imageUrl(base: string, opts: ImageOpts = {}): string {
  const q: string[] = [];
  if (opts.w) q.push(`w=${Math.round(opts.w)}`);
  if (opts.mask) q.push("mask=1");
  if (opts.crop) q.push("crop=1");
  return q.length ? `${base}${base.includes("?") ? "&" : "?"}${q.join("&")}` : base;
}

export const nodeImageBase = (src: string, sym: string, i = 0) => `${srcPath(src)}/image/${encodeURIComponent(sym)}/${i}`;

// --- which nodes have images (one /imaged call per source, refreshed on change) ----

const IMAGED_TTL_MS = 10_000;
const imagedCache = new Map<string, {key: string; at: number; syms: Set<string> | null; pending?: Promise<void>}>();

export function useImaged(srcs: {id: string; rev: number}[]): Record<string, Set<string> | null> {
  const [, bump] = useState(0);
  const key = srcs.map(s => `${s.id}@${s.rev}`).join(",");
  useEffect(() => {
    let alive = true;
    for (const s of srcs) {
      const cur = imagedCache.get(s.id);
      const k = String(s.rev);
      if (cur && (cur.pending || (cur.key === k) || Date.now() - cur.at < IMAGED_TTL_MS)) continue;
      const entry = {key: k, at: Date.now(), syms: cur?.syms ?? null} as {key: string; at: number; syms: Set<string> | null; pending?: Promise<void>};
      entry.pending = getJSON<{syms: string[]}>(`${srcPath(s.id)}/imaged`)
        .then(r => void (entry.syms = new Set(r.syms)))
        .catch(() => void (entry.syms = entry.syms ?? new Set()))
        .finally(() => {
          entry.pending = undefined;
          entry.at = Date.now();
          if (alive) bump(n => n + 1);
        });
      imagedCache.set(s.id, entry);
    }
    return () => {
      alive = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key]);
  const out: Record<string, Set<string> | null> = {};
  for (const s of srcs) out[s.id] = imagedCache.get(s.id)?.syms ?? null;
  return out;
}

/** Test hook. */
export function _resetImagedCache() {
  imagedCache.clear();
}

// --- search thumbnail -------------------------------------------------------------

function useVisible<T extends Element>(): [RefObject<T | null>, boolean] {
  const ref = useRef<T>(null);
  const [seen, setSeen] = useState(typeof IntersectionObserver === "undefined");
  useEffect(() => {
    const el = ref.current;
    if (seen || !el) return;
    const io = new IntersectionObserver(
      es => {
        if (es.some(e => e.isIntersecting)) {
          setSeen(true);
          io.disconnect();
        }
      },
      {rootMargin: "80px"},
    );
    io.observe(el);
    return () => io.disconnect();
  }, [seen]);
  return [ref, seen];
}

/** 40 px object thumbnail (cropped, mask overlaid); placeholder when the node has
 *  no image (or `has` is still unknown), loaded only once scrolled into view. */
export function SearchThumb({src, sym, has, color}: {src: string; sym: string; has: boolean | null; color: string}) {
  const [ref, visible] = useVisible<HTMLSpanElement>();
  const [failed, setFailed] = useState(false);
  const show = has && visible && !failed;
  return (
    <span ref={ref} className={`sr-thumb ${show ? "" : "sr-thumb-empty"}`} style={{borderColor: color}} aria-hidden="true">
      {show ? (
        <img src={withToken(imageUrl(nodeImageBase(src, sym), {w: 96, mask: true, crop: true}))} alt="" decoding="async" onError={() => setFailed(true)} />
      ) : (
        <span className="sr-thumb-dot" style={{background: color}} />
      )}
    </span>
  );
}

// --- Inspect: hero image + strip + lightbox, mask toggle --------------------------

const MASK_KEY = "adt4.inspect.mask";

function loadMaskPref(): boolean {
  try {
    return localStorage.getItem(MASK_KEY) !== "0";
  } catch {
    return true;
  }
}

export function ObjectImages({images, masks, title, isObject}: {images: string[]; masks?: boolean[]; title: string; isObject: boolean}) {
  const [lightbox, setLightbox] = useState<number | null>(null);
  const [maskOn, setMaskOn] = useState(loadMaskPref);
  useEffect(() => setLightbox(null), [images]);
  if (!images.length) return null;
  const anyMask = Boolean(masks?.some(Boolean));
  const withMask = (i: number) => maskOn && Boolean(masks?.[i]);
  const setMask = (v: boolean) => {
    setMaskOn(v);
    try {
      localStorage.setItem(MASK_KEY, v ? "1" : "0");
    } catch {
      /* private mode */
    }
  };
  const hero = isObject ? 0 : -1;
  const rest = images.map((u, i) => [u, i] as const).filter(([, i]) => i !== hero);
  return (
    <div className="obj-images">
      <div className="row gap between obj-images-head">
        <h4 className="sub-title">Images ({images.length})</h4>
        {anyMask && <Toggle checked={maskOn} onChange={setMask} label="Mask overlay" />}
      </div>
      {hero === 0 && (
        <button className="obj-hero" onClick={() => setLightbox(0)} aria-label="Best image, open full size">
          <img src={withToken(imageUrl(images[0], {w: 720, mask: withMask(0)}))} alt={`${title} best view`} />
          <span className="obj-hero-tag mono">best view{masks?.[0] ? (maskOn ? " · mask" : "") : ""}</span>
        </button>
      )}
      {rest.length > 0 && (
        <div className="image-strip">
          {rest.map(([u, i]) => (
            <button key={u} className="thumb" onClick={() => setLightbox(i)} aria-label={`Image ${i + 1}`}>
              <img src={withToken(imageUrl(u, {w: 240, mask: withMask(i)}))} loading="lazy" alt={`${title} frame ${i + 1}`} />
            </button>
          ))}
        </div>
      )}
      {lightbox !== null && images[lightbox] && (
        <Modal title={`${title}: frame ${lightbox + 1}/${images.length}`} onClose={() => setLightbox(null)} wide>
          <img className="lightbox-img" src={withToken(imageUrl(images[lightbox], {mask: withMask(lightbox)}))} alt="" />
          <div className="row gap center">
            <button className="btn btn-sm" disabled={lightbox === 0} onClick={() => setLightbox(lightbox - 1)}>
              Previous
            </button>
            {anyMask && <Toggle checked={maskOn} onChange={setMask} label="Mask" />}
            <button className="btn btn-sm" disabled={lightbox >= images.length - 1} onClick={() => setLightbox(lightbox + 1)}>
              Next
            </button>
          </div>
        </Modal>
      )}
    </div>
  );
}
