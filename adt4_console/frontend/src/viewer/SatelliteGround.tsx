// Satellite imagery under the map (API.md §14): XYZ tiles from the console's
// cache placed in the map frame through the GPS anchor's affine fit. Drawn
// just under the lowest graph point so the mesh and graph stay on top in 3D;
// in the top view it is the planning backdrop.
import {useEffect, useMemo, useState} from "react";
import * as THREE from "three";
import {useThree} from "@react-three/fiber";
import {withToken} from "../lib/auth";
import {fetchWithProgress} from "../lib/api";
import {lonLatToTile, pickZoom, tilesForBox, type TileQuad} from "../lib/geo";
import {useMission} from "../stores/mission";
import {beginTask, dropTask, endTask, progressTask} from "../stores/loading";

const MARGIN_M = 60;

// imagery on the load bar (stores/loading): tiles in / tiles wanted
const IMAGERY_TASK = "imagery";
const imagery = {want: new Set<string>(), got: new Set<string>()};
function imageryWant(keys: string[]) {
  imagery.want = new Set(keys);
  imagery.got = new Set([...imagery.got].filter(k => imagery.want.has(k)));
  if (!keys.length || imagery.got.size >= keys.length) return dropTask(IMAGERY_TASK);
  beginTask(IMAGERY_TASK, "Imagery", "count", keys.length);
  progressTask(IMAGERY_TASK, imagery.got.size);
}
function imageryGot(key: string) {
  if (!imagery.want.has(key) || imagery.got.has(key)) return;
  imagery.got.add(key);
  progressTask(IMAGERY_TASK, imagery.got.size);
  if (imagery.got.size >= imagery.want.size) endTask(IMAGERY_TASK);
}

function Tile({q, layer, z, opacity}: {q: TileQuad; layer: string; z: number; opacity: number}) {
  const invalidate = useThree(s => s.invalidate);
  const [tex, setTex] = useState<THREE.Texture | null>(null);
  useEffect(() => {
    // fetched (not TextureLoader) so the bytes count on the load bar's throughput meter
    const ctl = new AbortController();
    let t: THREE.Texture | null = null;
    void (async () => {
      try {
        const body = await fetchWithProgress(withToken(`api/geo/tiles/${layer}/${q.z}/${q.x}/${q.y}`), {signal: ctl.signal});
        const bmp = await createImageBitmap(new Blob([body as BlobPart]), {imageOrientation: "flipY"});
        if (ctl.signal.aborted) return bmp.close();
        const tx = new THREE.Texture(bmp);
        tx.flipY = false; // flipped while decoding (ImageBitmaps ignore flipY)
        tx.colorSpace = THREE.SRGBColorSpace;
        tx.anisotropy = 4;
        tx.needsUpdate = true;
        t = tx;
        setTex(tx);
        invalidate();
      } catch {
        // not cached (and no live fetching): leave the hole
      } finally {
        imageryGot(q.key);
      }
    })();
    return () => {
      ctl.abort();
      t?.dispose();
      (t?.image as ImageBitmap | undefined)?.close?.();
    };
  }, [layer, q.z, q.x, q.y, invalidate]);
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const c = q.corners; // NW NE SE SW
    g.setAttribute("position", new THREE.Float32BufferAttribute([c[0][0], c[0][1], z, c[1][0], c[1][1], z, c[2][0], c[2][1], z, c[3][0], c[3][1], z], 3));
    g.setAttribute("uv", new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
    g.setIndex([0, 3, 1, 1, 3, 2]);
    return g;
  }, [q, z]);
  useEffect(() => () => geom.dispose(), [geom]);
  if (!tex) return null;
  return (
    <mesh geometry={geom} renderOrder={-10}>
      <meshBasicMaterial map={tex} transparent={opacity < 1} opacity={opacity} depthWrite={false} side={THREE.DoubleSide} toneMapped={false} />
    </mesh>
  );
}

export function SatelliteGround({floorZ}: {floorZ: number | null}) {
  const geo = useMission(s => s.geo);
  const on = useMission(s => s.satellite);
  const opacity = useMission(s => s.satOpacity);
  const layers = useMission(s => s.layers);
  const world = useMission(s => s.world);
  const layer = layers.find(l => l.id === "esri_world_imagery") ?? layers[0];
  const tiles = useMemo(() => {
    if (!geo || !layer || !world?.bounds) return [];
    const b = world.bounds;
    const box = {min: [b[0] - MARGIN_M, b[1] - MARGIN_M], max: [b[2] + MARGIN_M, b[3] + MARGIN_M]};
    const zmax = layer.zmax ?? 19;
    const z = pickZoom(geo, box, zmax, Math.max(layer.zmin ?? 12, 10));
    const all = tilesForBox(geo, box, z) ?? [];
    // offline: only what the cache can have (its manifest's box), no 404 storm
    const bb = layer.bbox_lonlat;
    if (layer.live || !bb) return all;
    const [x0, y1] = lonLatToTile(bb[0], bb[1], z);
    const [x1, y0] = lonLatToTile(bb[2], bb[3], z);
    return all.filter(q => q.x >= Math.floor(x0) && q.x <= Math.floor(x1) && q.y >= Math.floor(y0) && q.y <= Math.floor(y1));
  }, [geo, layer, world]);
  const shown = on && !!layer;
  useEffect(() => {
    imageryWant(shown ? tiles.map(q => q.key) : []);
  }, [shown, tiles]);
  useEffect(() => () => imageryWant([]), []);
  if (!on || !layer || !tiles.length) return null;
  const z = (floorZ ?? geo?.anchor.z0 ?? 0) - 0.3;
  return (
    <group>
      {tiles.map(q => (
        <Tile key={q.key} q={q} layer={layer.id} z={z} opacity={opacity} />
      ))}
    </group>
  );
}
