import {describe, expect, it} from "vitest";
import {KeyGate, RateMeter, chunkInit, decoderConfig, h264Decoder, parseVideoFrame, pickCodec} from "../src/lib/video";
import {cropI420, i420Size, i420ToRgba} from "../src/lib/yuv";

const f = (key: boolean, extra: Partial<{codec: "h264" | "vp8" | "jpeg"; width: number; height: number}> = {}) => ({key, codec: "h264" as const, width: 640, height: 360, ...extra});

describe("KeyGate", () => {
  it("waits for the first key frame", () => {
    const g = new KeyGate();
    expect(g.accept(f(false), 1).action).toBe("skip");
    expect(g.accept(f(false), 2).action).toBe("skip");
    expect(g.accept(f(true), 3)).toEqual({action: "decode", reconfigure: true});
    expect(g.accept(f(false), 4)).toEqual({action: "decode", reconfigure: false});
    expect(g.skipped).toBe(2);
  });
  it("skips deltas after a seq gap until the next key frame", () => {
    const g = new KeyGate();
    g.accept(f(true), 10);
    g.accept(f(false), 11);
    expect(g.accept(f(false), 13).action).toBe("skip"); // 12 lost
    expect(g.waitingKey).toBe(true);
    expect(g.accept(f(false), 14).action).toBe("skip");
    expect(g.accept(f(true), 20)).toEqual({action: "decode", reconfigure: false}); // gap but key: fine
    expect(g.accept(f(false), 21).action).toBe("decode");
  });
  it("reconfigures on the key frame of a new codec or size", () => {
    const g = new KeyGate();
    g.accept(f(true), 1);
    expect(g.accept(f(false, {width: 320, height: 180}), 2).action).toBe("skip");
    expect(g.accept(f(true, {width: 320, height: 180}), 3)).toEqual({action: "decode", reconfigure: true});
    expect(g.accept(f(true, {codec: "vp8", width: 320, height: 180}), 4).reconfigure).toBe(true);
  });
  it("broken() waits for a key frame and reconfigures on it", () => {
    const g = new KeyGate();
    g.accept(f(true), 1);
    g.broken();
    expect(g.accept(f(false), 2).action).toBe("skip");
    expect(g.accept(f(true), 3)).toEqual({action: "decode", reconfigure: true});
  });
});

describe("video helpers", () => {
  it("measures fps and kbps over a window", () => {
    const m = new RateMeter(2000);
    for (let i = 0; i < 10; i++) m.add(1000, i * 200); // 5 fps, 1000 B each
    const r = m.rates(2000);
    expect(r.fps).toBeCloseTo(5, 0);
    expect(r.kbps).toBeCloseTo(40, -1);
    expect(m.rates(10_000)).toEqual({fps: 0, kbps: 0});
    // a burst after a quiet spell is averaged over the whole window
    m.add(1000, 20_000);
    m.add(1000, 20_100);
    expect(m.rates(20_200).fps).toBeCloseTo(1, 1);
  });
  it("builds chunks and decoder configs (Annex-B h264: no description)", () => {
    const fr = parseVideoFrame({camera: "rgb", codec: "h264", seq: 7, t: 1, key: true, width: 640, height: 360, fps: 5, data: new Uint8Array([0, 0, 0, 1, 9])})!;
    expect(fr.key).toBe(true);
    expect(chunkInit(fr)).toEqual({type: "key", timestamp: 7000, data: fr.data});
    expect(chunkInit({...fr, key: false}).type).toBe("delta");
    expect(decoderConfig("h264")).toEqual({codec: "avc1.42E01E", optimizeForLatency: true});
    expect(decoderConfig("h264")).not.toHaveProperty("description");
    expect(decoderConfig("jpeg")).toBeNull();
    expect(parseVideoFrame({codec: "h264", seq: 1})).toBeNull();
  });
  it("picks h264, then vp8, then jpeg", async () => {
    const dec = (ok: string[]) => ({isConfigSupported: async (c: VideoDecoderConfig) => ({supported: ok.includes(c.codec)})});
    expect(await pickCodec(dec(["avc1.42E01E", "vp8"]), false)).toBe("h264");
    expect(await pickCodec(dec(["vp8"]), false)).toBe("vp8");
    expect(await pickCodec(dec([]), false)).toBe("jpeg");
    expect(await pickCodec(undefined, false)).toBe("jpeg");
  });
  it("falls back to WASM h264 without WebCodecs (plain-http pages)", async () => {
    const dec = (ok: string[]) => ({isConfigSupported: async (c: VideoDecoderConfig) => ({supported: ok.includes(c.codec)})});
    expect(await pickCodec(undefined, true)).toBe("h264");
    expect(await pickCodec(dec(["vp8"]), true)).toBe("h264");
    expect(await h264Decoder(undefined, true)).toBe("wasm");
    expect(await h264Decoder(dec(["avc1.42E01E"]), true)).toBe("webcodecs");
    expect(await h264Decoder(dec([]), false)).toBeNull();
  });
});

describe("skew-corrected lag", () => {
  it("uses the console-mapped capture time and the browser offset", async () => {
    const {frameLagMs} = await import("../src/lib/video");
    const {bestOffset} = await import("../src/lib/clock");
    // browser clock runs 5 s behind the console; capture 120 ms ago (console clock)
    const off = bestOffset([
      {t0: 100.0, t1: 100.4, server: 105.3}, // slow sample: rtt 0.4
      {t0: 200.0, t1: 200.02, server: 205.01}, // best: offset 5.0
    ]);
    expect(off).toBeCloseTo(5.0, 6);
    const nowMs = 300_000; // browser clock
    expect(frameLagMs({tBase: 304.88}, nowMs, off)).toBeCloseTo(120, 3);
    expect(frameLagMs({tBase: null}, nowMs, off)).toBeNull();
    expect(frameLagMs({tBase: 304.88}, nowMs, null)).toBeNull();
  });
});

describe("I420 helpers", () => {
  // a padded 4x4 picture whose plane bytes encode (plane, row, col)
  const padded = (pw: number, ph: number) => {
    const b = new Uint8Array(i420Size(pw, ph));
    let o = 0;
    for (let y = 0; y < ph; y++) for (let x = 0; x < pw; x++) b[o++] = 10 * y + x;
    for (const plane of [100, 200]) for (let y = 0; y < ph / 2; y++) for (let x = 0; x < pw / 2; x++) b[o++] = plane + 10 * y + x;
    return b;
  };
  it("crops the macroblock padding off the right and bottom", () => {
    const out = cropI420(padded(6, 4), 6, 4, 4, 2);
    expect(Array.from(out)).toEqual([0, 1, 2, 3, 10, 11, 12, 13, 100, 101, 200, 201]);
  });
  it("is a no-op view when nothing is padded, and rejects odd / larger crops", () => {
    const src = padded(4, 4);
    expect(cropI420(src, 4, 4, 4, 4)).toEqual(src);
    expect(() => cropI420(src, 4, 4, 3, 4)).toThrow();
    expect(() => cropI420(src, 4, 4, 4, 6)).toThrow();
  });
  it("converts BT.601 limited range", () => {
    const px = (y: number, u: number, v: number) => Array.from(i420ToRgba(new Uint8Array([y, y, y, y, u, v]), 2, 2).slice(0, 4));
    expect(px(16, 128, 128)).toEqual([0, 0, 0, 255]);
    expect(px(235, 128, 128)).toEqual([255, 255, 255, 255]);
    const red = px(81, 90, 240); // BT.601 limited-range red
    expect(red[0]).toBeGreaterThan(250);
    expect(red[1]).toBeLessThan(5);
    expect(red[2]).toBeLessThan(5);
  });
});
