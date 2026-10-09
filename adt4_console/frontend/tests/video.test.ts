import {describe, expect, it} from "vitest";
import {KeyGate, RateMeter, chunkInit, decoderConfig, parseVideoFrame, pickCodec} from "../src/lib/video";

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
    expect(await pickCodec(dec(["avc1.42E01E", "vp8"]))).toBe("h264");
    expect(await pickCodec(dec(["vp8"]))).toBe("vp8");
    expect(await pickCodec(dec([]))).toBe("jpeg");
    expect(await pickCodec(undefined)).toBe("jpeg");
  });
});
