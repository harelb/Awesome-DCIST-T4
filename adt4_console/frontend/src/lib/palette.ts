// Deterministic label -> colour palette (high contrast on a dark background).
const PALETTE: [number, number, number][] = [
  [230, 25, 75], [60, 180, 75], [255, 225, 25], [67, 99, 216], [245, 130, 49],
  [145, 30, 180], [66, 212, 244], [240, 50, 230], [191, 239, 69], [250, 190, 212],
  [70, 153, 144], [220, 190, 255], [154, 99, 36], [255, 250, 200], [128, 0, 0],
  [170, 255, 195], [128, 128, 0], [255, 216, 177], [0, 117, 255], [169, 169, 169],
];

export function labelColor(label: number): [number, number, number] {
  if (label < 0) return [200, 200, 200];
  // multiplicative hash so neighbouring ids get distant palette entries
  const h = (Math.imul(label + 1, 2654435761) >>> 0) % PALETTE.length;
  return PALETTE[h];
}

export function nodeColor(label: number, color: [number, number, number]): [number, number, number] {
  if (color[0] || color[1] || color[2]) return color;
  return labelColor(label);
}

export const LAYER_COLORS: Record<string, [number, number, number]> = {
  agents: [255, 120, 40],
  subkeyframes: [255, 190, 130],
  places: [70, 200, 255],
  rooms: [240, 80, 200],
  buildings: [255, 255, 255],
  segments: [180, 180, 180],
  other: [180, 180, 180],
};

export const rgbCss = (c: [number, number, number]) => `rgb(${c[0]}, ${c[1]}, ${c[2]})`;

/** Per-source tint for overlaying several graph sources. */
export const SOURCE_TINTS = ["#4fc3f7", "#ffb74d", "#ba68c8", "#81c784", "#f06292"];
