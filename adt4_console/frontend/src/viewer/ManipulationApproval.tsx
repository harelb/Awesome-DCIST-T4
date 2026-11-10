import {useEffect, useRef, useState, type MouseEvent} from "react";
import {ApiError, getJSON, postJSON} from "../lib/api";
import {withToken} from "../lib/auth";
import "./manipulationApproval.css";

export interface ManipulationImage {
  index: number;
  name?: string;
  source?: string;
  width: number;
  height: number;
  url: string;
  error?: string;
  /** Colourised depth registered to this image's pixels (transparent where unmeasured). */
  depth_url?: string;
  /** GET with ?x=&y= (image pixels): measured depth and the walk-to ground-plane estimate. */
  point_query_url?: string;
  /** The request carries the camera/ground geometry for this image's walk-to estimate. */
  ground_estimate?: boolean;
  /** Set when this camera's walk estimates should not be trusted (the gripper camera). */
  ground_estimate_warning?: string;
  depth_error?: string;
}

export interface GraspSelection {
  image_index: number;
  x: number;
  y: number;
}

export interface ManipulationRequest {
  id: string;
  robot: string;
  received_at: number;
  expires_at: number;
  images: ManipulationImage[];
  selection: GraspSelection | null;
  simulated?: boolean;
  status?: string;
  /** Near and far ends of the depth colour scale, metres. */
  depth_range_m?: [number, number];
  walk_to?: {standoff_m: number; max_distance_m: number};
}

interface GroundEstimate {
  hit: boolean;
  /** Horizontal distance from the robot to where the pixel's ray meets the ground. */
  distance_m?: number;
  /** How far Walk To would move: distance minus standoff, capped. 0 = turn only. */
  walk_m?: number;
  capped?: boolean;
}

interface PointReading {
  image_index: number;
  x: number;
  y: number;
  has_depth: boolean;
  depth_m: number | null;
  ground: GroundEstimate | null;
}

/** What Walk To would do for this point. It comes ONLY from the ground-plane estimate. */
export function describeWalk(reading: PointReading | null | undefined, unreliable = false): string {
  const ground = reading?.ground;
  if (!ground) return "";
  const caveat = unreliable ? " (unreliable for this camera)" : "";
  if (!ground.hit || ground.distance_m === undefined) return `Walk To (ground plane): ray misses the floor${caveat}`;
  const move = !ground.walk_m ? "turns only" : `walks ${ground.walk_m.toFixed(2)} m${ground.capped ? " (capped)" : ""}`;
  return `Walk To (ground plane): ${ground.distance_m.toFixed(2)} m away → ${move}${caveat}`;
}

/** Measured depth is shown for reference; Walk To never uses it. */
export function describeDepth(reading: PointReading | null | undefined): string {
  if (!reading?.has_depth) return "";
  return reading.depth_m === null ? "measured depth: none" : `measured depth ${reading.depth_m.toFixed(2)} m (display only)`;
}

export function describePoint(reading: PointReading | null | undefined, unreliable = false): string {
  return [describeWalk(reading, unreliable), describeDepth(reading)].filter(Boolean).join(" · ");
}

interface Pending {
  available: boolean;
  error: string | null;
  requests: ManipulationRequest[];
}

interface ResponseBody {
  approve: boolean;
  image_index?: number;
  image_x?: number;
  image_y?: number;
  /** approve=false + walk_to: walk toward the selected pixel, then ask again with new images. */
  walk_to?: boolean;
  /** Heading control for walk_to: the ground pixel to stand on, facing the target. */
  stand_x?: number;
  stand_y?: number;
}

type Answer = "approve" | "reject" | "walk";

/** Heading (stand point) selection for Walk To is a feature flag, off by default. */
export const WALK_TO_HEADING_SELECTION = false;

interface ResponseResult {
  ok: boolean;
  status: string;
  robot: string;
  request_id: string;
}

/** Use the displayed image bounds, rather than viewport or CSS pixel coordinates. */
export function imagePixel(clientX: number, clientY: number,
  rect: Pick<DOMRect, "left" | "top" | "width" | "height">,
  width: number, height: number): {x: number; y: number} | null {
  if (rect.width <= 0 || rect.height <= 0 || width <= 0 || height <= 0) return null;
  const nx = (clientX - rect.left) / rect.width;
  const ny = (clientY - rect.top) / rect.height;
  if (nx < 0 || nx >= 1 || ny < 0 || ny >= 1) return null;
  return {x: Math.min(width - 1, Math.floor(nx * width)),
    y: Math.min(height - 1, Math.floor(ny * height))};
}

export function validSelection(request: ManipulationRequest,
  selection: GraspSelection | null): selection is GraspSelection {
  if (!selection) return false;
  const image = request.images.find(item => item.index === selection.image_index);
  return Boolean(image && !image.error && Number.isInteger(selection.x) && Number.isInteger(selection.y)
    && selection.x >= 0 && selection.y >= 0 && selection.x < image.width && selection.y < image.height);
}

function detail(error: unknown): string {
  return error instanceof ApiError ? error.detail : error instanceof Error ? error.message : String(error);
}

/** Global host: pending requests are independent of the currently selected console page. */
export function ManipulationApprovalHost() {
  const [pending, setPending] = useState<Pending>({available: false, error: null, requests: []});
  const [connected, setConnected] = useState(false);
  const [fetchError, setFetchError] = useState<string | null>(null);
  const [dismissed, setDismissed] = useState<Set<string>>(new Set());
  const [answered, setAnswered] = useState<Set<string>>(new Set());
  const [notice, setNotice] = useState<string | null>(null);
  const [demoBusy, setDemoBusy] = useState(false);

  useEffect(() => {
    const controller = new AbortController();
    let timer: number | undefined;
    const poll = async () => {
      try {
        const state = await getJSON<Pending>("api/manipulation/pending", controller.signal);
        if (controller.signal.aborted) return;
        setPending(state);
        setConnected(true);
        setFetchError(null);
      } catch (error) {
        if (controller.signal.aborted) return;
        setConnected(false);
        setFetchError(detail(error));
      }
      if (!controller.signal.aborted) timer = window.setTimeout(poll, 1000);
    };
    void poll();
    return () => { controller.abort(); if (timer !== undefined) window.clearTimeout(timer); };
  }, []);

  const requests = pending.requests.filter(request => !answered.has(request.id));
  const active = requests.find(request => !dismissed.has(request.id));
  const respond = async (request: ManipulationRequest, body: ResponseBody) => {
    const result = await postJSON<ResponseResult>(
      `api/manipulation/requests/${encodeURIComponent(request.id)}/respond`, body);
    setAnswered(old => new Set([...old, request.id]));
    const kind = body.approve ? "approval" : body.walk_to ? "walk-to" : "rejection";
    setNotice(result.status === "simulated"
      ? "Simulation response recorded. No robot message was sent."
      : body.walk_to
        ? `${request.robot}: walk-to request published. The robot will move, take new images and ask again.`
        : `${request.robot}: ${kind} response published. This does not confirm action completion.`);
  };

  const demo = async () => {
    setDemoBusy(true);
    try {
      await postJSON("api/manipulation/demo");
      const state = await getJSON<Pending>("api/manipulation/pending");
      setPending(state);
      setConnected(true);
      setFetchError(null);
      setDismissed(new Set());
    } catch (error) {
      setNotice(detail(error));
    } finally {
      setDemoBusy(false);
    }
  };

  return <>
    {notice && <div className="manipulation-notice" role="status">
      <span>{notice}</span>
      <button type="button" className="icon-btn" aria-label="Dismiss manipulation status" onClick={() => setNotice(null)}>×</button>
    </div>}
    {!active && requests.length > 0 && <button type="button" className="manipulation-pending btn btn-primary"
      onClick={() => setDismissed(new Set())}>
      Review manipulation approval ({requests.length})
    </button>}
    {!active && requests.length === 0 && <button type="button" className="manipulation-demo btn"
      disabled={demoBusy} onClick={() => void demo()} title="Open a simulated manipulation request; no robot command is sent">
      {demoBusy ? "Opening test…" : "Test manipulation dialog"}
    </button>}
    {active && <ManipulationApprovalDialog key={active.id} request={active}
      available={connected && (pending.available || Boolean(active.simulated))}
      serviceError={fetchError || (!active.simulated ? pending.error : null)}
      onDismiss={() => setDismissed(old => new Set([...old, active.id]))}
      onRespond={body => respond(active, body)}
      headingSelection={WALK_TO_HEADING_SELECTION} />}
  </>;
}

export function ManipulationApprovalDialog({request, available = true, serviceError,
  onDismiss, onRespond, headingSelection = false}: {
  request: ManipulationRequest;
  available?: boolean;
  serviceError?: string | null;
  onDismiss: () => void;
  onRespond: (body: ResponseBody) => Promise<void>;
  /** Offer a stand-point (heading) choice for Walk To. Off by default. */
  headingSelection?: boolean;
}) {
  const initial = validSelection(request, request.selection) ? request.selection : null;
  const initialIndex = request.images.findIndex(image => image.index === initial?.image_index);
  const [shown, setShown] = useState(Math.max(0, initialIndex));
  const [selection, setSelection] = useState<GraspSelection | null>(initial);
  const [override, setOverride] = useState(false);
  const [standPoint, setStandPoint] = useState<GraspSelection | null>(null);
  const [pickingStand, setPickingStand] = useState(false);
  const [showDepth, setShowDepth] = useState(true);
  const [cursorDepth, setCursorDepth] = useState<PointReading | null>(null);
  const [selectionDepth, setSelectionDepth] = useState<PointReading | null>(null);
  const depthQuery = useRef({last: 0, busy: false});
  const [loaded, setLoaded] = useState(false);
  const [loadedImages, setLoadedImages] = useState<Set<number>>(new Set());
  const [imageError, setImageError] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const close = useRef<HTMLButtonElement>(null);
  const dialog = useRef<HTMLDivElement>(null);
  const image = request.images[shown];
  const selectedImage = request.images.find(item => item.index === selection?.image_index);
  const selectedHere = selection?.image_index === image?.index ? selection : null;
  const standHere = standPoint?.image_index === image?.index ? standPoint : null;
  // A stand point is only meaningful on the same image as the target pixel.
  const standUsable = headingSelection && validSelection(request, standPoint)
    && standPoint?.image_index === selection?.image_index;
  const expired = Number.isFinite(request.expires_at) && request.expires_at * 1000 <= Date.now();
  const canApprove = available && !expired && !busy && validSelection(request, selection)
    && Boolean(selectedImage && !selectedImage.error)
    && Boolean(selection && loadedImages.has(selection.image_index))
    && (selection?.image_index !== image?.index || (loaded && !imageError));
  const canWalk = canApprove;
  // Put the planned move on the button itself, so it is clear what Walk To will do.
  const selectedWalk = selection && selectionDepth && selectionDepth.image_index === selection.image_index
    && selectionDepth.x === selection.x && selectionDepth.y === selection.y ? selectionDepth.ground : null;
  const walkLabel = !selectedWalk ? ""
    : !selectedWalk.hit ? " (no floor hit)"
    : ` (${selectedWalk.walk_m ? `${selectedWalk.walk_m.toFixed(1)} m` : "turn only"}, ground plane${
      selectedImage?.ground_estimate_warning ? ", unreliable" : ""})`;

  useEffect(() => {
    const previous = document.activeElement as HTMLElement | null;
    close.current?.focus();
    return () => previous?.focus();
  }, []);

  useEffect(() => {
    setLoaded(false); setImageError(null); setOverride(false); setPickingStand(false); setCursorDepth(null);
  }, [image?.url]);

  // Depth under the chosen point, so the operator knows how far it is before answering.
  useEffect(() => {
    setSelectionDepth(null);
    const target = request.images.find(item => item.index === selection?.image_index);
    if (!selection || !target?.point_query_url) return;
    const controller = new AbortController();
    getJSON<PointReading>(`${target.point_query_url}?x=${selection.x}&y=${selection.y}`, controller.signal)
      .then(reading => { if (!controller.signal.aborted) setSelectionDepth(reading); })
      .catch(() => undefined);
    return () => controller.abort();
  }, [request, selection]);

  const hover = (event: MouseEvent<HTMLImageElement>) => {
    if (!image?.point_query_url || !loaded) return;
    const now = Date.now();
    if (depthQuery.current.busy || now - depthQuery.current.last < 120) return;
    const pixel = imagePixel(event.clientX, event.clientY, event.currentTarget.getBoundingClientRect(), image.width, image.height);
    if (!pixel) return;
    depthQuery.current = {last: now, busy: true};
    const index = image.index;
    getJSON<PointReading>(`${image.point_query_url}?x=${pixel.x}&y=${pixel.y}`)
      .then(reading => setCursorDepth(current => (image.index === index ? reading : current)))
      .catch(() => undefined)
      .finally(() => { depthQuery.current.busy = false; });
  };

  const answer = async (kind: Answer) => {
    if (busy || !available || expired) return;
    if (kind === "approve" && !canApprove) return;
    if (kind === "walk" && !canWalk) return;
    setBusy(true);
    setError(null);
    try {
      let body: ResponseBody = {approve: false};
      if (kind !== "reject" && selection) {
        body = {approve: kind === "approve", image_index: selection.image_index, image_x: selection.x, image_y: selection.y};
        if (kind === "walk") {
          body.walk_to = true;
          if (standUsable && standPoint) { body.stand_x = standPoint.x; body.stand_y = standPoint.y; }
        }
      }
      await onRespond(body);
    } catch (failure) {
      setError(detail(failure));
    } finally {
      setBusy(false);
    }
  };

  const pick = (event: MouseEvent<HTMLImageElement>) => {
    if ((!override && !pickingStand) || busy || !image || !loaded || imageError) return;
    const pixel = imagePixel(event.clientX, event.clientY, event.currentTarget.getBoundingClientRect(), image.width, image.height);
    if (!pixel) return;
    if (override) { setSelection({image_index: image.index, ...pixel}); setOverride(false); }
    else { setStandPoint({image_index: image.index, ...pixel}); setPickingStand(false); }
  };

  const reset = () => {
    setSelection(initial);
    setStandPoint(null);
    setShown(Math.max(0, initialIndex));
    setOverride(false);
    setPickingStand(false);
  };

  return <div className="manipulation-backdrop" onMouseDown={event => {
    if (event.target === event.currentTarget && !busy) onDismiss();
  }}>
    <div ref={dialog} className="manipulation-dialog" role="dialog" aria-modal="true"
      aria-labelledby="manipulation-approval-title" onKeyDown={event => {
        event.stopPropagation();
        if (event.key === "Escape" && !busy) { event.preventDefault(); onDismiss(); }
        if (event.key === "Tab") {
          const controls = dialog.current?.querySelectorAll<HTMLElement>("button:not(:disabled), input:not(:disabled)");
          if (!controls?.length) return;
          const first = controls[0], last = controls[controls.length - 1];
          if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last.focus(); }
          else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first.focus(); }
        }
      }}>
      <header className="manipulation-head">
        <div><h2 id="manipulation-approval-title">Manipulation approval</h2>
          <p className="muted"><strong>{request.robot}</strong> is waiting for a grasp decision.</p></div>
        <button type="button" ref={close} className="icon-btn" aria-label="Close without responding"
          disabled={busy} onClick={onDismiss}>×</button>
      </header>
      {request.simulated && <div className="manipulation-simulation">SIMULATION — no robot response</div>}
      <div className="manipulation-body">
        <p>Approving lets the waiting robot action continue, including pick or drag.
          Walk To instead moves the robot toward the selected ground/object point, then asks again with new images.</p>
        <div className="manipulation-image-nav">
          <button type="button" className="btn" disabled={busy || shown <= 0}
            aria-label="Previous candidate image" onClick={() => setShown(index => index - 1)}>← Previous</button>
          <span><strong>{image?.name || image?.source || "No candidate image"}</strong>
            {image && <span className="muted"> · {shown + 1} / {request.images.length} · {image.width} × {image.height}</span>}</span>
          <button type="button" className="btn" disabled={busy || shown >= request.images.length - 1}
            aria-label="Next candidate image" onClick={() => setShown(index => index + 1)}>Next →</button>
        </div>
        {image && !image.error ? <div className="manipulation-image-area">
          <div className={`manipulation-image-wrap ${override || pickingStand ? "is-picking" : ""}`}>
            <img key={image.url} src={withToken(image.url)} draggable={false}
              alt={`Grasp candidate ${image.name || image.source || image.index}`}
              onLoad={() => { setLoaded(true); setLoadedImages(old => new Set([...old, image.index])); }}
              onError={() => {
                setImageError("Unable to load this candidate image");
                setLoadedImages(old => new Set([...old].filter(index => index !== image.index)));
              }}
              onClick={pick} onMouseMove={hover} onMouseLeave={() => setCursorDepth(null)} />
            {loaded && showDepth && image.depth_url && <img className="manipulation-depth" aria-hidden="true" alt=""
              data-testid="depth-overlay" draggable={false} src={withToken(image.depth_url)} />}
            {loaded && selectedHere && <span className="manipulation-crosshair" aria-hidden="true"
              style={{left: `${(selectedHere.x + 0.5) / image.width * 100}%`, top: `${(selectedHere.y + 0.5) / image.height * 100}%`}} />}
            {loaded && headingSelection && standHere && <span className="manipulation-crosshair is-stand" aria-hidden="true"
              data-testid="stand-marker"
              style={{left: `${(standHere.x + 0.5) / image.width * 100}%`, top: `${(standHere.y + 0.5) / image.height * 100}%`}} />}
          </div>
          {!loaded && !imageError && <span className="muted">Loading candidate image…</span>}
        </div> : <p className="manipulation-error" role="alert">{image?.error || "The request has no usable images."}</p>}
        {imageError && <p className="manipulation-error" role="alert">{imageError}</p>}
        <div className="manipulation-point-controls">
          <label><input type="checkbox" checked={override} disabled={busy || !loaded || Boolean(imageError || image?.error)}
            onChange={event => { setOverride(event.target.checked); if (event.target.checked) setPickingStand(false); }} /> Override selected point</label>
          {headingSelection && <label><input type="checkbox" checked={pickingStand}
            disabled={busy || !loaded || Boolean(imageError || image?.error)}
            onChange={event => { setPickingStand(event.target.checked); if (event.target.checked) setOverride(false); }} /> Choose stand point (heading)</label>}
          <button type="button" className="btn" disabled={busy || (!initial && !standPoint)} onClick={reset}>Reset to detection</button>
        </div>
        {image?.point_query_url && <div className="manipulation-depth-controls">
          {image.depth_url && <>
            <label><input type="checkbox" checked={showDepth} onChange={event => setShowDepth(event.target.checked)} /> Show depth</label>
            <span className="manipulation-depth-legend" aria-hidden="true" />
            <span className="muted">{(request.depth_range_m ?? [0.3, 6])[0]} m near → {(request.depth_range_m ?? [0.3, 6])[1]} m far</span>
          </>}
          <span data-testid="depth-readout">{cursorDepth && describePoint(cursorDepth, Boolean(image.ground_estimate_warning))
            ? `Cursor (${cursorDepth.x}, ${cursorDepth.y}): ${describePoint(cursorDepth, Boolean(image.ground_estimate_warning))}`
            : image.depth_url || image.ground_estimate ? "Hover the image for distances" : "No depth or walk estimate for this image"}</span>
        </div>}
        {image?.ground_estimate_warning && <p className="manipulation-error" role="alert" data-testid="estimate-warning">
          {image.ground_estimate_warning}</p>}
        {image?.ground_estimate && request.walk_to && <p className="manipulation-hint muted" data-testid="walk-note">
          Walk To plans only from the ground-plane estimate, which assumes the point is on the floor and is
          unreliable near the horizon. The depth overlay and measured depth are for reference and are not used.
          The robot's body center stops {request.walk_to.standoff_m.toFixed(1)} m short of the point and moves
          at most {request.walk_to.max_distance_m.toFixed(1)} m per walk.</p>}
        {image && !image.depth_url && image.depth_error && <p className="manipulation-hint">Depth unavailable: {image.depth_error}</p>}
        {override && <p className="manipulation-hint">Click the grasp point, or the ground/object point to walk toward, on this image.</p>}
        {pickingStand && <p className="manipulation-hint">Click the ground pixel the robot should stand on. It will face the selected point from there.</p>}
        <p className="manipulation-selection" data-testid="grasp-selection">
          {selection && selectedImage ? <>Selected: <strong>{selectedImage.name || selectedImage.source || `image ${selectedImage.index + 1}`}</strong>
            {` · pixel (${selection.x}, ${selection.y})`}
            {selectionDepth && selectionDepth.image_index === selection.image_index && selectionDepth.x === selection.x
              && selectionDepth.y === selection.y && describePoint(selectionDepth, Boolean(selectedImage.ground_estimate_warning))
              && ` · ${describePoint(selectionDepth, Boolean(selectedImage.ground_estimate_warning))}`}</> : "No point selected. Enable the override and click an image."}
        </p>
        {headingSelection && standPoint && <p className="manipulation-selection" data-testid="stand-selection">
          {standUsable ? `Stand point · pixel (${standPoint.x}, ${standPoint.y})`
            : "Stand point is on a different image than the selected point and will be ignored."}
        </p>}
        {(serviceError || !available) && <p className="manipulation-error" role="alert">{serviceError || "Approval connection unavailable. No response can be sent."}</p>}
        {expired && <p className="manipulation-error" role="alert">This request has expired. Wait for a new robot request.</p>}
        {error && <p className="manipulation-error" role="alert">{error}</p>}
      </div>
      <footer className="manipulation-foot">
        <button type="button" className="btn" disabled={busy} onClick={onDismiss}>Close without response</button>
        <div className="row gap">
          <button type="button" className="btn btn-danger" disabled={busy || !available || expired}
            onClick={() => void answer("reject")}>Reject grasp</button>
          <button type="button" className="btn" disabled={!canWalk}
            title="Do not grasp yet: walk toward the selected point, then take new images and ask again"
            onClick={() => void answer("walk")}>Walk to selected point{walkLabel}</button>
          <button type="button" className="btn btn-primary" disabled={!canApprove}
            onClick={() => void answer("approve")}>{busy ? "Sending response…" : "Approve selected grasp"}</button>
        </div>
      </footer>
    </div>
  </div>;
}
