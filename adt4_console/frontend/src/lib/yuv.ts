// I420 (yuv420p) helpers for the WASM h264 path. Pure, no DOM.

/** Bytes in a tight I420 picture (both dimensions even). */
export const i420Size = (w: number, h: number) => w * h + 2 * (w / 2) * (h / 2);

/**
 * Crops a decoded picture to the stream's size. h264bsd hands back the whole
 * macroblock grid (640x360 decodes as 640x368) and ignores the SPS crop, which
 * for our encoder always trims the right / bottom edge.
 */
export function cropI420(src: Uint8Array, pw: number, ph: number, w: number, h: number): Uint8Array {
  if (w > pw || h > ph || w % 2 || h % 2) throw new Error(`bad crop ${w}x${h} of ${pw}x${ph}`);
  if (w === pw && h === ph) return src.subarray(0, i420Size(w, h));
  const out = new Uint8Array(i420Size(w, h));
  let o = 0;
  for (let y = 0; y < h; y++, o += w) out.set(src.subarray(y * pw, y * pw + w), o);
  const cw = w / 2, ch = h / 2, cpw = pw / 2, cph = ph / 2;
  for (const base of [pw * ph, pw * ph + cpw * cph]) {
    for (let y = 0; y < ch; y++, o += cw) out.set(src.subarray(base + y * cpw, base + y * cpw + cw), o);
  }
  return out;
}

/** BT.601 limited range (what ffmpeg's rgb24 → yuv420p produces) to RGBA. */
export function i420ToRgba(src: Uint8Array, w: number, h: number, out: Uint8ClampedArray = new Uint8ClampedArray(w * h * 4)): Uint8ClampedArray {
  const cw = w / 2;
  const uOff = w * h;
  const vOff = uOff + cw * (h / 2);
  for (let y = 0; y < h; y++) {
    const crow = (y >> 1) * cw;
    for (let x = 0; x < w; x++) {
      const Y = 1.164 * (src[y * w + x] - 16);
      const U = src[uOff + crow + (x >> 1)] - 128;
      const V = src[vOff + crow + (x >> 1)] - 128;
      const i = (y * w + x) * 4;
      out[i] = Y + 1.596 * V;
      out[i + 1] = Y - 0.392 * U - 0.813 * V;
      out[i + 2] = Y + 2.017 * U;
      out[i + 3] = 255;
    }
  }
  return out;
}
