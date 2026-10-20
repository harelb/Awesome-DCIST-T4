import {useLayoutEffect, useMemo, useRef} from "react";
import * as THREE from "three";
import {useThree} from "@react-three/fiber";
import {Html, Line} from "@react-three/drei";
import {LineSegments2} from "three/examples/jsm/lines/LineSegments2.js";
import {LineSegmentsGeometry} from "three/examples/jsm/lines/LineSegmentsGeometry.js";
import {LineMaterial} from "three/examples/jsm/lines/LineMaterial.js";
import type {GraphLayout, LayerBuffers, TextLabel, VizStyle} from "./layout";
import {defaultEdgeOn, EDGE_COLORS, HYDRA_EDGES, PLACE_BOUNDARY} from "./layout";

/** Displayed node positions per source, used by the click picker. */
export const pickRegistry = new Map<string, Map<string, LayerBuffers>>();

const nextPow2 = (n: number) => 2 ** Math.ceil(Math.log2(Math.max(n, 16)));

const sphere = new THREE.IcosahedronGeometry(1, 1);
const box = new THREE.BoxGeometry(1.4, 1.4, 1.4);

function NodeLayer({layer}: {layer: LayerBuffers}) {
  const capacity = nextPow2(layer.syms.length);
  const ref = useRef<THREE.InstancedMesh>(null);
  const invalidate = useThree(s => s.invalidate);
  const geom = layer.shape === "box" ? box : sphere;
  const material = useMemo(
    () => new THREE.MeshLambertMaterial({color: "#ffffff", transparent: layer.opacity < 1, opacity: layer.opacity}),
    [layer.opacity],
  );
  useLayoutEffect(() => () => material.dispose(), [material]);
  useLayoutEffect(() => {
    const m = ref.current;
    if (!m) return;
    const mat = new THREE.Matrix4();
    const col = new THREE.Color();
    const s = layer.size;
    for (let i = 0; i < layer.syms.length; i++) {
      mat.makeScale(s, s, s);
      mat.setPosition(layer.positions[i * 3], layer.positions[i * 3 + 1], layer.positions[i * 3 + 2]);
      m.setMatrixAt(i, mat);
      col.setRGB(layer.colors[i * 3], layer.colors[i * 3 + 1], layer.colors[i * 3 + 2], THREE.SRGBColorSpace);
      m.setColorAt(i, col);
    }
    m.count = layer.syms.length;
    m.instanceMatrix.needsUpdate = true;
    if (m.instanceColor) m.instanceColor.needsUpdate = true;
    m.computeBoundingSphere();
    invalidate();
  }, [layer, invalidate]);
  return <instancedMesh key={`${capacity}/${layer.shape}/${layer.opacity}`} ref={ref} args={[geom, material, capacity]} frustumCulled={false} />;
}

/** hydra RegionGrowingBoundaryPlugin boundary: one merged LINE_LIST with
 *  per-vertex state colours, 0.07 m wide in world units like the RViz marker. */
function PlaceBoundary({positions, colors}: {positions: Float32Array; colors: Float32Array}) {
  const obj = useMemo(() => {
    const g = new LineSegmentsGeometry();
    g.setPositions(positions);
    g.setColors(colors);
    const m = new LineMaterial({
      vertexColors: true,
      worldUnits: true,
      linewidth: PLACE_BOUNDARY.lineWidth,
      transparent: PLACE_BOUNDARY.lineAlpha < 1,
      opacity: PLACE_BOUNDARY.lineAlpha,
    });
    const l = new LineSegments2(g, m);
    l.frustumCulled = false;
    l.raycast = () => {};
    return l;
  }, [positions, colors]);
  useLayoutEffect(
    () => () => {
      obj.geometry.dispose();
      obj.material.dispose();
    },
    [obj],
  );
  return <primitive object={obj} />;
}

/** Translucent place-region fill (hydra RegionGrowingBoundaryPlugin, fill_alpha 0.3). */
function PlaceFill({positions, colors}: {positions: Float32Array; colors: Float32Array}) {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    g.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    return g;
  }, [positions, colors]);
  useLayoutEffect(() => () => geom.dispose(), [geom]);
  return (
    <mesh geometry={geom} frustumCulled={false} renderOrder={-1}>
      <meshBasicMaterial vertexColors transparent opacity={PLACE_BOUNDARY.fillAlpha} side={THREE.DoubleSide} depthWrite={false} />
    </mesh>
  );
}

function TextLabels({texts}: {texts: TextLabel[]}) {
  return (
    <>
      {texts.map((t, i) => (
        <Html key={i} position={t.pos} center zIndexRange={[5, 0]} className="viz-text-wrap" style={{pointerEvents: "none"}}>
          <div className={`viz-text viz-text-${t.kind}`}>{t.text}</div>
        </Html>
      ))}
    </>
  );
}

function Segments({positions, colors, color, opacity = 1}: {positions: Float32Array; colors?: Float32Array; color?: string; opacity?: number}) {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    if (colors) g.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    return g;
  }, [positions, colors]);
  useLayoutEffect(() => () => geom.dispose(), [geom]);
  return (
    <lineSegments geometry={geom} frustumCulled={false}>
      <lineBasicMaterial vertexColors={Boolean(colors)} color={colors ? "#ffffff" : color} transparent={opacity < 1} opacity={opacity} />
    </lineSegments>
  );
}

export function GraphLayers({src, layout, showBBoxes, showTrajectory, trajectory, edgeKinds, style = "classic", showText = false}: {
  src: string;
  layout: GraphLayout;
  showBBoxes: boolean;
  showTrajectory: boolean;
  trajectory: [number, number, number][] | null;
  edgeKinds: Record<string, boolean>;
  style?: VizStyle;
  showText?: boolean;
}) {
  const hydra = style === "hydra";
  useLayoutEffect(() => {
    pickRegistry.set(src, new Map(layout.layers.map(l => [l.key, l])));
    return () => void pickRegistry.delete(src);
  }, [src, layout]);
  const trajPoints = useMemo(() => {
    if (trajectory && trajectory.length > 1) return trajectory;
    if (!layout.agentPath) return null;
    const out: [number, number, number][] = [];
    for (let i = 0; i < layout.agentPath.length; i += 3) out.push([layout.agentPath[i], layout.agentPath[i + 1], layout.agentPath[i + 2]]);
    return out;
  }, [trajectory, layout.agentPath]);
  return (
    <group name={`graph:${src}`}>
      {layout.layers.map(l => (
        <NodeLayer key={l.key} layer={l} />
      ))}
      {showBBoxes && layout.bboxes.positions.length > 0 && <Segments positions={layout.bboxes.positions} colors={layout.bboxes.colors} opacity={0.9} />}
      {hydra && layout.placeFill && <PlaceFill positions={layout.placeFill.positions} colors={layout.placeFill.colors} />}
      {hydra && layout.placeBoundary && <PlaceBoundary positions={layout.placeBoundary.positions} colors={layout.placeBoundary.colors} />}
      {Object.entries(layout.edges).map(([kind, pos]) =>
        (edgeKinds[kind] ?? defaultEdgeOn(kind, style)) && kind !== "agents-agents" ? (
          hydra ? (
            <Segments
              key={kind}
              positions={pos}
              colors={layout.edgeColors[kind]}
              color={HYDRA_EDGES[kind]?.color ?? "#000000"}
              opacity={HYDRA_EDGES[kind]?.alpha ?? 0.5}
            />
          ) : (
            <Segments key={kind} positions={pos} color={EDGE_COLORS[kind] ?? "#7f8b96"} opacity={kind === "places-places" ? 0.55 : 0.35} />
          )
        ) : null,
      )}
      {showTrajectory && trajPoints && trajPoints.length > 1 && (
        <Line points={trajPoints} color={hydra ? HYDRA_EDGES["agents-agents"].color : EDGE_COLORS["agents-agents"]} lineWidth={hydra ? 2 : 2.5} />
      )}
      {showText && layout.texts.length > 0 && <TextLabels texts={layout.texts} />}
    </group>
  );
}
