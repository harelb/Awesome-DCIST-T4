// Map areas and map edit mode in the 3D/top view: the map's areas (Room nodes:
// boundary polygon + name; Layers › "Area boundaries"), "colour members by
// area" while editing (member discs in the area's colour), the selection
// (rings), the lasso being drawn and the subtree focus (the edited graph dimmed
// like an out-of-focus robot, the area's descendants drawn bright on top).
import {useEffect, useMemo} from "react";
import * as THREE from "three";
import {useFrame, useThree} from "@react-three/fiber";
import {Html, Line} from "@react-three/drei";
import {useMapEdit} from "../stores/mapEdit";
import {useViewStyle} from "../stores/viewStyle";
import {useViewer} from "../stores/viewer";
import {sourceMatrix} from "../stores/robots";
import {applyPoint} from "../lib/frameAlign";
import {pickRegistry} from "./GraphLayers";
import {applyDim, DIM} from "./DimGroup";
import {descendants, uiSym} from "../lib/mapEdit";
import {PALETTE} from "../lib/palette";

type V3 = [number, number, number];
const ORDER = 40;

/** Displayed position of every node of `src` (what the picker uses), global frame. */
function usePositions(src: string | null): Map<string, V3> {
  const rev = useViewer(s => (src ? s.sources[src]?.rev : 0));
  const enabled = useViewer(s => (src ? s.sources[src]?.enabled : false));
  const layoutKey = useViewer(s => `${s.flat}/${s.stackHeight}/${s.vizStyle}`);
  return useMemo(() => {
    const out = new Map<string, V3>();
    const layers = src && enabled ? pickRegistry.get(src) : null;
    if (!layers) return out;
    const m = sourceMatrix(src!);
    for (const l of layers.values()) {
      for (let i = 0; i < l.syms.length; i++) {
        const p: V3 = [l.positions[i * 3], l.positions[i * 3 + 1], l.positions[i * 3 + 2]];
        out.set(l.syms[i], m ? applyPoint(m, p) : p);
      }
    }
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [src, rev, enabled, layoutKey]);
}

export const regionColor = (i: number): string => {
  const [r, g, b] = PALETTE[(i * 5 + 2) % PALETTE.length];
  return `rgb(${r},${g},${b})`;
};

function Points({pts, color, size}: {pts: V3[]; color: string | Float32Array; size: number}) {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pts.flat()), 3));
    if (typeof color !== "string") g.setAttribute("color", new THREE.BufferAttribute(color, 3));
    return g;
  }, [pts, color]);
  useEffect(() => () => geom.dispose(), [geom]);
  if (!pts.length) return null;
  return (
    <points geometry={geom} renderOrder={ORDER} frustumCulled={false}>
      <pointsMaterial size={size} sizeAttenuation={false} vertexColors={typeof color !== "string"} color={typeof color === "string" ? color : "#ffffff"} depthTest={false} transparent />
    </points>
  );
}

/** Dim the edited source's graph while a subtree is focused (DimGroup's applyDim). */
function FocusDimmer({src, on}: {src: string; on: boolean}) {
  const scene = useThree(s => s.scene);
  const invalidate = useThree(s => s.invalidate);
  const each = (f: number) => {
    const g = scene.getObjectByName(`graph:${src}`);
    g?.traverse(o => {
      const mat = (o as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
      if (!mat) return;
      for (const m of Array.isArray(mat) ? mat : [mat]) applyDim(m, f);
    });
  };
  useFrame(() => {
    if (on) each(DIM);
  });
  useEffect(() => {
    invalidate();
    if (!on) each(1);
    return () => each(1);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [on, src]);
  return null;
}

export function EditOverlay() {
  const {on, src, selection, lasso, tool, regions, focusRoot} = useMapEdit();
  const showRegions = useViewStyle(s => s.regionHulls);
  const colorByRegion = useViewStyle(s => s.regionColorPlaces) && on;
  // the graph's own text labels already name each Room node: one label per area
  const graphText = useViewer(s => s.showText);
  const pos = usePositions(src);
  const model = useViewer(s => (src ? s.sources[src]?.model : null));
  const rev = useViewer(s => (src ? s.sources[src]?.rev : 0));
  const ground = useMemo(() => {
    let z = Infinity;
    for (const p of pos.values()) z = Math.min(z, p[2]);
    return Number.isFinite(z) ? z : 0;
  }, [pos]);

  const sel = useMemo(() => selection.map(s => pos.get(s)).filter((p): p is V3 => !!p), [selection, pos]);

  // region members, coloured per region
  const members = useMemo(() => {
    if (!colorByRegion || !model) return null;
    const pts: V3[] = [];
    const cols: number[] = [];
    regions.forEach((r, i) => {
      const c = new THREE.Color(regionColor(i));
      for (const s of descendants(model, uiSym(r.symbol))) {
        const p = pos.get(s);
        if (!p) continue;
        pts.push(p);
        cols.push(c.r, c.g, c.b);
      }
    });
    return {pts, cols: new Float32Array(cols)};
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [colorByRegion, model, rev, regions, pos]);

  const focus = useMemo(() => {
    if (!focusRoot || !model) return null;
    const d = descendants(model, focusRoot);
    return [...d].map(s => pos.get(s)).filter((p): p is V3 => !!p);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [focusRoot, model, rev, pos]);

  if (!src) return null;
  const z = ground + 0.15;
  return (
    <group name="map-edit-overlay">
      <FocusDimmer src={src} on={!!focusRoot} />
      {showRegions &&
        regions.map((r, i) =>
          r.polygon && r.polygon.length >= 3 ? (
            <group key={r.symbol}>
              <Line points={[...r.polygon, r.polygon[0]].map(p => [p[0], p[1], z] as V3)} color={regionColor(i)} lineWidth={2.5} dashed dashSize={0.8} gapSize={0.4} depthTest={false} renderOrder={ORDER} />
              {!graphText && (
                <Html position={[r.polygon.reduce((a, p) => a + p[0], 0) / r.polygon.length, r.polygon.reduce((a, p) => a + p[1], 0) / r.polygon.length, z]} center zIndexRange={[20, 0]} style={{pointerEvents: "none"}}>
                  <div className="region-tag" style={{borderColor: regionColor(i)}}>
                    {r.name || "unnamed area"}
                  </div>
                </Html>
              )}
            </group>
          ) : null,
        )}
      {members && <Points pts={members.pts} color={members.cols} size={9} />}
      {focus && <Points pts={focus} color="#ffd23f" size={8} />}
      {on &&
        sel.slice(0, 400).map((p, i) => (
          <mesh key={i} position={p} renderOrder={ORDER}>
            <torusGeometry args={[0.7, 0.08, 6, 24]} />
            <meshBasicMaterial color="#2fd3ff" depthTest={false} transparent />
          </mesh>
        ))}
      {on && tool === "lasso" && lasso.length > 0 && (
        <>
          <Line points={[...lasso, ...(lasso.length > 2 ? [lasso[0]] : [])].map(p => [p[0], p[1], z] as V3)} color="#2fd3ff" lineWidth={2} dashed dashSize={0.5} gapSize={0.3} depthTest={false} renderOrder={ORDER} />
          {lasso.map((p, i) => (
            <mesh key={i} position={[p[0], p[1], z]} renderOrder={ORDER}>
              <sphereGeometry args={[0.3, 10, 8]} />
              <meshBasicMaterial color="#2fd3ff" depthTest={false} transparent />
            </mesh>
          ))}
        </>
      )}
    </group>
  );
}
