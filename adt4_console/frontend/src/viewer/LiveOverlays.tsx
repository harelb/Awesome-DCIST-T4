// 3D overlays for live robots: occupancy grid (textured plane) and executor
// paths in each robot's colour (smooth = lighter dashed, MLP = robot colour,
// MLP target = darker marker), matching the RViz fleet config. Lidar points
// (on demand, off by default) coloured by height.
import {useEffect, useMemo, useState} from "react";
import * as THREE from "three";
import {Line} from "@react-three/drei";
import {useFrameAlign, useLiveRobots, useRobots} from "../stores/robots";
import {gridOpacityOf, gridShown, useLidarReenable, useOverlays, watchGrid} from "../stores/overlays";
import {ApiError, errorText, getMsgpackIfChanged} from "../lib/api";
import {LIDAR_POLL_MS, LIDAR_REENABLE_S, decodeLidar, lidarLayerFrame, type LidarFrame} from "../lib/lidar";
import {Toggle} from "../components/ui";
import {lift, type DecodedGrid} from "../lib/overlays";
import {shade, useRobotColorOf} from "../lib/palette";
import {AlignedGroup} from "./AlignedGroup";
import {DimGroup} from "./DimGroup";
import {useBackground} from "./background";
import {contrastFor, readableOn} from "./style";

/** Executor overlay colours derived from a robot's colour. On a light 3D
 *  background (`bg`) the shades are darkened just enough to stay visible. */
export function pathColors(robotHex: string, bg?: string) {
  if (bg && contrastFor(bg).light) {
    const mlp = readableOn(robotHex, bg, 3);
    return {smooth: readableOn(shade(robotHex, 1.25), bg, 2), mlp, target: shade(mlp, 0.6)};
  }
  return {smooth: shade(robotHex, 1.45), mlp: robotHex, target: shade(robotHex, 0.6)};
}


function GridPlane({grid, opacity}: {grid: DecodedGrid; opacity: number}) {
  const texture = useMemo(() => {
    const t = new THREE.DataTexture(grid.rgba, grid.width, grid.height, THREE.RGBAFormat);
    t.magFilter = THREE.NearestFilter;
    t.minFilter = THREE.LinearFilter;
    t.generateMipmaps = false;
    t.colorSpace = THREE.SRGBColorSpace;
    t.needsUpdate = true;
    return t;
  }, [grid]);
  useEffect(() => () => texture.dispose(), [texture]);
  const w = grid.width * grid.resolution;
  const h = grid.height * grid.resolution;
  const [x, y, z, yaw] = grid.origin;
  // OccupancyGrid origin = corner of cell (0, 0); the plane is centred, so shift by half its size
  return (
    <group position={[x, y, z + 0.02]} rotation={[0, 0, yaw]}>
      <mesh position={[w / 2, h / 2, 0]} renderOrder={1}>
        <planeGeometry args={[w, h]} />
        <meshBasicMaterial map={texture} transparent opacity={opacity} depthWrite={false} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
}

function GridLayer({robot}: {robot: string}) {
  const grid = useOverlays(s => s.grids[robot]);
  const opacity = useOverlays(s => gridOpacityOf(s, robot));
  useEffect(() => watchGrid(robot), [robot]);
  return grid ? (
    <AlignedGroup robot={robot} frame={grid.frame}>
      <GridPlane grid={grid} opacity={opacity} />
    </AlignedGroup>
  ) : null;
}

function LidarPoints({frame}: {frame: LidarFrame}) {
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(frame.positions, 3));
    g.setAttribute("color", new THREE.BufferAttribute(frame.colors, 3));
    g.computeBoundingSphere();
    return g;
  }, [frame]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  const material = useMemo(() => new THREE.PointsMaterial({size: 0.06, sizeAttenuation: true, vertexColors: true, depthWrite: false, transparent: true, opacity: 0.9}), []);
  useEffect(() => () => material.dispose(), [material]);
  const [x, y, z, qx, qy, qz, qw] = frame.origin;
  const quat = useMemo(() => new THREE.Quaternion(qx, qy, qz, qw).normalize(), [qx, qy, qz, qw]);
  return (
    <group position={[x, y, z]} quaternion={quat}>
      <points geometry={geometry} material={material} renderOrder={2} />
    </group>
  );
}

/** Polls the robot's lidar frame (~1 Hz, ETag) while mounted; asks the robot
 *  again when it reports the stream off (agent restart, page reload). */
function LidarLayer({robot}: {robot: string}) {
  const lidar = useRobots(s => s.state[robot]?.lidar);
  const poseFrame = useRobots(s => s.state[robot]?.pose?.frame);
  const fa = useFrameAlign(robot);
  const [frame, setFrame] = useState<LidarFrame | null>(null);
  const available = lidar?.available !== false;
  useEffect(() => {
    if (!available) return;
    const ctl = new AbortController();
    let etag: string | null = null;
    let timer: number | undefined;
    const tick = async () => {
      try {
        const r = await getMsgpackIfChanged(`api/live/${encodeURIComponent(robot)}/lidar`, etag, ctl.signal);
        if (ctl.signal.aborted) return;
        if (r.changed) {
          etag = r.etag;
          setFrame(decodeLidar(r.data));
        }
        useOverlays.setState(s => (s.lidarError[robot] ? {lidarError: {...s.lidarError, [robot]: null}} : s));
      } catch (e) {
        if (ctl.signal.aborted) return;
        // 404: nothing received yet
        const msg = e instanceof ApiError && e.status === 404 ? null : errorText(e);
        useOverlays.setState(s => ({lidarError: {...s.lidarError, [robot]: msg}}));
      }
      if (!ctl.signal.aborted) timer = window.setTimeout(tick, LIDAR_POLL_MS);
    };
    void tick();
    return () => {
      ctl.abort();
      window.clearTimeout(timer);
    };
  }, [robot, available]);
  // wanted but off on the robot: ask now and every LIDAR_REENABLE_S until on
  useLidarReenable(robot, lidar, LIDAR_REENABLE_S);
  if (!frame || !available) return null;
  return (
    <AlignedGroup robot={robot} frame={lidarLayerFrame(frame.frame, poseFrame, fa)}>
      <LidarPoints frame={frame} />
    </AlignedGroup>
  );
}

function PathLayer({robot}: {robot: string}) {
  const paths = useRobots(s => s.paths[robot]);
  const {showSmooth, showMlp, showTarget} = useOverlays();
  const c = pathColors(useRobotColorOf()(robot), useBackground());
  if (!paths) return null;
  const smooth = lift(paths.smooth, 0.15);
  const mlp = lift(paths.mlp, 0.2);
  const t = paths.target;
  return (
    <AlignedGroup robot={robot} frame={paths.frame}>
      {showSmooth && smooth.length > 1 && <Line points={smooth} color={c.smooth} lineWidth={2.5} dashed dashSize={0.35} gapSize={0.2} />}
      {showMlp && mlp.length > 1 && <Line points={mlp} color={c.mlp} lineWidth={3} />}
      {showTarget && t && (
        <group position={[t[0], t[1], t[2] + 0.25]}>
          <mesh>
            <sphereGeometry args={[0.18, 16, 10]} />
            <meshBasicMaterial color={c.target} />
          </mesh>
          <mesh position={[0, 0, -0.2]} rotation={[0, 0, 0]}>
            <ringGeometry args={[0.35, 0.45, 32]} />
            <meshBasicMaterial color={c.target} side={THREE.DoubleSide} transparent opacity={0.8} />
          </mesh>
        </group>
      )}
    </AlignedGroup>
  );
}

export function LiveOverlays() {
  const robots = useLiveRobots();
  const gridOn = useOverlays(s => s.gridOn);
  const lidarOn = useOverlays(s => s.lidarOn);
  return (
    <>
      {robots.map(r => (
        <DimGroup key={r} robot={r}>
          {(gridOn[r] ?? true) && <GridLayer robot={r} />}
          {lidarOn[r] && <LidarLayer robot={r} />}
          <PathLayer robot={r} />
        </DimGroup>
      ))}
    </>
  );
}

/** Left-panel section: per-robot occupancy toggle + opacity, path toggles. */
export function LiveOverlaysPanel() {
  const robots = useLiveRobots();
  const st = useOverlays();
  const paths = useRobots(s => s.paths);
  const colorOf = useRobotColorOf();
  const bg = useBackground();
  if (!robots.length) return null;
  // legend swatches: first live robot's shades (each robot draws in its own colour)
  const legend = pathColors(colorOf(robots[0]), bg);
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Live overlays</h4>
      </header>
      {robots.map(r => {
        const g = st.grids[r];
        const err = st.gridError[r];
        const on = gridShown(st, r);
        const op = gridOpacityOf(st, r);
        return (
          <div key={r} className="overlay-robot">
            <Toggle
              checked={on}
              onChange={v => st.set({gridOn: {...st.gridOn, [r]: v}})}
              label={
                <span>
                  <span className="swatch" style={{background: colorOf(r)}} /> {r}: occupancy
                </span>
              }
            />
            {on && (
              <>
                <label className="slider-row">
                  <span>Opacity</span>
                  <input type="range" min={0.05} max={1} step={0.05} value={op} aria-label={`${r} occupancy opacity`} onChange={e => st.set({gridOpacity: {...st.gridOpacity, [r]: Number(e.target.value)}})} />
                  <span className="mono small">{Math.round(op * 100)}%</span>
                </label>
                <div className="muted small mono">
                  {g ? `${g.width}x${g.height} @ ${g.resolution.toFixed(2)} m${g.frame ? ` · ${g.frame}` : ""}` : err ? (/no occupancy grid/i.test(err) ? "no grid received yet" : err) : "loading grid"}
                </div>
              </>
            )}
            {paths[r] && (
              <div className="muted small mono">
                paths: smooth {paths[r].smooth.length} · mlp {paths[r].mlp.length} · target {paths[r].target ? "yes" : "none"}
              </div>
            )}
          </div>
        );
      })}
      <div className="muted small">Paths are drawn per robot in that robot's colour.</div>
      <div className="row gap-sm wrap overlay-path-toggles">
        <label className="mini-check">
          <input type="checkbox" checked={st.showSmooth} onChange={e => st.set({showSmooth: e.target.checked})} />
          <span className="swatch line" style={{background: legend.smooth}} /> smooth path (lighter)
        </label>
        <label className="mini-check">
          <input type="checkbox" checked={st.showMlp} onChange={e => st.set({showMlp: e.target.checked})} />
          <span className="swatch line" style={{background: legend.mlp}} /> MLP path (robot colour)
        </label>
        <label className="mini-check">
          <input type="checkbox" checked={st.showTarget} onChange={e => st.set({showTarget: e.target.checked})} />
          <span className="swatch" style={{background: legend.target}} /> MLP target (darker)
        </label>
      </div>
    </section>
  );
}
