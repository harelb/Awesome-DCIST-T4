import {useLayoutEffect, useMemo, useRef} from "react";
import * as THREE from "three";
import {useThree} from "@react-three/fiber";
import {Line} from "@react-three/drei";
import type {GraphLayout, LayerBuffers} from "./layout";
import {DEFAULT_EDGE_ON, EDGE_COLORS} from "./layout";

/** Displayed node positions per source, used by the click picker. */
export const pickRegistry = new Map<string, Map<string, LayerBuffers>>();

const nextPow2 = (n: number) => 2 ** Math.ceil(Math.log2(Math.max(n, 16)));

const sphere = new THREE.IcosahedronGeometry(1, 1);
const box = new THREE.BoxGeometry(1.4, 1.4, 1.4);

function NodeLayer({layer}: {layer: LayerBuffers}) {
  const capacity = nextPow2(layer.syms.length);
  const ref = useRef<THREE.InstancedMesh>(null);
  const invalidate = useThree(s => s.invalidate);
  const geom = layer.info.kind === "objects" ? box : sphere;
  const material = useMemo(() => new THREE.MeshLambertMaterial({color: "#ffffff"}), []);
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
  return <instancedMesh key={capacity} ref={ref} args={[geom, material, capacity]} frustumCulled={false} />;
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

export function GraphLayers({src, layout, showBBoxes, showTrajectory, trajectory, edgeKinds}: {
  src: string;
  layout: GraphLayout;
  showBBoxes: boolean;
  showTrajectory: boolean;
  trajectory: [number, number, number][] | null;
  edgeKinds: Record<string, boolean>;
}) {
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
      {Object.entries(layout.edges).map(([kind, pos]) =>
        (edgeKinds[kind] ?? DEFAULT_EDGE_ON(kind)) && kind !== "agents-agents" ? (
          <Segments key={kind} positions={pos} color={EDGE_COLORS[kind] ?? "#7f8b96"} opacity={kind === "places-places" ? 0.55 : 0.35} />
        ) : null,
      )}
      {showTrajectory && trajPoints && trajPoints.length > 1 && <Line points={trajPoints} color={EDGE_COLORS["agents-agents"]} lineWidth={2.5} />}
    </group>
  );
}
