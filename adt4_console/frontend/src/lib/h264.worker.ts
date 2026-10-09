// Software h264 decoder (tinyh264 = h264bsd in WASM) for pages without
// WebCodecs, i.e. the console opened over plain http from another machine.
// Baseline profile only, which is what the robot's x264 produces.
//
// in:  {type: "decode", id, data, width, height}  one Annex-B access unit
//      {type: "reset"}                             drop the decoder state
// out: {type: "ready"} | {type: "error", id?, message}
//      {type: "frame", id, width, height, data, decodeMs}  tight I420
//      {type: "none", id}                          no picture from this AU
/// <reference lib="webworker" />
import TinyH264, {type TinyH264Module} from "tinyh264/es/TinyH264";
import {cropI420} from "./yuv";

const PIC_RDY = 1;
const ERROR = 3;

class Decoder {
  private storage: number;
  private pW: number;
  private pH: number;
  private pPic: number;
  private buf = 0;
  private bufLen = 0;

  constructor(private m: TinyH264Module) {
    this.storage = m._h264bsdAlloc();
    this.pW = m._malloc(4);
    this.pH = m._malloc(4);
    this.pPic = m._malloc(4);
    m._h264bsdInit(this.storage, 0);
  }

  /** Decodes one access unit; returns the padded picture or null. */
  decode(au: Uint8Array): {pic: Uint8Array; w: number; h: number} | null {
    const m = this.m;
    if (au.byteLength > this.bufLen) {
      if (this.buf) m._free(this.buf);
      this.bufLen = Math.max(au.byteLength, 256 * 1024);
      this.buf = m._malloc(this.bufLen);
    }
    m.HEAPU8.set(au, this.buf);
    const rc = m._h264bsdDecode(this.storage, this.buf, au.byteLength, this.pPic, this.pW, this.pH);
    if (rc === PIC_RDY) {
      const w = m.getValue(this.pW, "i32");
      const h = m.getValue(this.pH, "i32");
      const p = m.getValue(this.pPic, "i8*");
      return {pic: m.HEAPU8.subarray(p, p + (w * h * 3) / 2), w, h};
    }
    if (rc >= ERROR) throw new Error(`h264bsd error ${rc}`);
    return null;
  }

  release() {
    const m = this.m;
    m._h264bsdShutdown(this.storage);
    m._h264bsdFree(this.storage);
    for (const p of [this.pW, this.pH, this.pPic, this.buf]) if (p) m._free(p);
  }
}

const scope = self as unknown as DedicatedWorkerGlobalScope;
let mod: TinyH264Module | null = null;
let dec: Decoder | null = null;
const pending: MessageEvent[] = [];

function handle(e: MessageEvent) {
  const msg = e.data as {type: string; id?: number; data?: ArrayBuffer; width?: number; height?: number};
  if (msg.type === "reset") {
    dec?.release();
    dec = null;
    return;
  }
  if (msg.type !== "decode" || !mod) return;
  const t0 = performance.now();
  try {
    dec ??= new Decoder(mod);
    const out = dec.decode(new Uint8Array(msg.data!));
    if (!out) return scope.postMessage({type: "none", id: msg.id});
    const w = Math.min(msg.width || out.w, out.w);
    const h = Math.min(msg.height || out.h, out.h);
    const crop = cropI420(out.pic, out.w, out.h, w, h);
    const pic = crop.buffer === mod.HEAPU8.buffer ? crop.slice() : crop; // never transfer the WASM heap
    scope.postMessage({type: "frame", id: msg.id, width: w, height: h, data: pic.buffer, decodeMs: performance.now() - t0}, [pic.buffer]);
  } catch (err) {
    dec?.release();
    dec = null; // a fresh decoder for the next key frame
    scope.postMessage({type: "error", id: msg.id, message: err instanceof Error ? err.message : String(err)});
  }
}

scope.onmessage = e => (mod ? handle(e) : pending.push(e));
TinyH264().then(
  m => {
    mod = m;
    scope.postMessage({type: "ready"});
    for (const e of pending.splice(0)) handle(e);
  },
  err => scope.postMessage({type: "error", message: `wasm init: ${String(err)}`}),
);
