// One live video stream → decoded pictures, for views that draw the picture
// themselves (the all-cameras grid crops one mosaic into many tiles).
// The same three paths as VideoView: WebCodecs VideoDecoder (h264 / vp8),
// the WASM h264 worker (plain-http pages without WebCodecs) and jpeg.
// `onPicture` gets a drawable source plus the stream frame it came from; the
// source is only valid during the call.
import {KeyGate, RateMeter, chunkInit, decoderConfig, type H264Decoder, type VideoFrame as StreamFrame} from "./video";
import type {WasmH264Decoder} from "./wasmH264";
import type {YuvRenderer} from "./yuvRenderer";

const WASM_BACKLOG = 4;
const WEBCODECS_BACKLOG = 8;

export interface DecoderOptions {
  h264: H264Decoder | null;
  onPicture: (src: CanvasImageSource, width: number, height: number, frame: StreamFrame) => void;
  onError?: (message: string) => void;
  /** lost sync: the caller asks the robot for a key frame (rate-limited here to 1/s) */
  onNeedKeyframe?: () => void;
}

export class StreamDecoder {
  readonly meter = new RateMeter();
  readonly gate = new KeyGate();
  last: StreamFrame | null = null;
  private decoder: VideoDecoder | null = null;
  private wasm: WasmH264Decoder | null = null;
  private wasmLoading = false;
  private renderer: YuvRenderer | null = null;
  private wasmCanvas: HTMLCanvasElement | null = null;
  private meta = new Map<number, StreamFrame>();
  private lastKfReq = -1e9;
  private closed = false;

  constructor(private opts: DecoderOptions) {}

  private remember(id: number, fr: StreamFrame) {
    this.meta.set(id, fr);
    if (this.meta.size > 64) this.meta.delete(this.meta.keys().next().value as number);
  }

  private take(id: number): StreamFrame | null {
    const fr = this.meta.get(id) ?? null;
    this.meta.delete(id);
    return fr;
  }

  private fail(message: string) {
    this.opts.onError?.(message);
    this.gate.broken();
  }

  private needKey() {
    const now = performance.now();
    if (now - this.lastKfReq > 1000) {
      this.lastKfReq = now;
      this.opts.onNeedKeyframe?.();
    }
  }

  private loadWasm() {
    if (this.wasmLoading) return;
    this.wasmLoading = true;
    void Promise.all([import("./wasmH264"), import("./yuvRenderer")]).then(
      ([{WasmH264Decoder}, {YuvRenderer}]) => {
        if (this.closed) return;
        this.wasm = new WasmH264Decoder(
          pic => {
            const fr = this.take(pic.id);
            if (this.closed || !fr) return;
            this.wasmCanvas ??= document.createElement("canvas");
            if (this.wasmCanvas.width !== pic.width || this.wasmCanvas.height !== pic.height) {
              this.wasmCanvas.width = pic.width;
              this.wasmCanvas.height = pic.height;
            }
            (this.renderer ??= new YuvRenderer(this.wasmCanvas)).draw(pic.data, pic.width, pic.height);
            this.opts.onPicture(this.wasmCanvas, pic.width, pic.height, fr);
          },
          (message, id) => {
            if (id !== undefined) this.take(id);
            this.fail(`decoder: ${message}`);
          },
        );
      },
      e => this.opts.onError?.(`decoder: ${e instanceof Error ? e.message : String(e)}`),
    );
  }

  private newDecoder(): VideoDecoder | null {
    if (typeof VideoDecoder === "undefined") return null;
    return new VideoDecoder({
      output: frame => {
        const fr = this.take(frame.timestamp);
        try {
          if (!this.closed && fr) this.opts.onPicture(frame, frame.displayWidth, frame.displayHeight, fr);
        } finally {
          frame.close();
        }
      },
      error: e => {
        this.fail(`decoder: ${e.message}`);
        this.decoder = null; // recreated on the next key frame
      },
    });
  }

  /** One WebSocket video frame; `wsSeq` is the hub's per-topic seq (gap detection). */
  push(fr: StreamFrame, wsSeq: number) {
    if (this.closed) return;
    this.meter.add(fr.data.byteLength, performance.now());
    this.last = fr;
    if (fr.codec === "jpeg") {
      void createImageBitmap(new Blob([fr.data as BlobPart], {type: "image/jpeg"})).then(
        bmp => {
          try {
            if (!this.closed) this.opts.onPicture(bmp, bmp.width, bmp.height, fr);
          } finally {
            bmp.close();
          }
        },
        () => this.opts.onError?.("bad jpeg frame"),
      );
      return;
    }
    const cfg = decoderConfig(fr.codec);
    const useWasm = fr.codec === "h264" && this.opts.h264 === "wasm";
    const webcodecs = typeof VideoDecoder !== "undefined" && !useWasm;
    if (!cfg || (!webcodecs && !useWasm)) {
      this.gate.skipped++;
      return;
    }
    if (useWasm && !this.wasm) {
      this.loadWasm();
      this.gate.broken();
    }
    if (this.decoder && this.decoder.decodeQueueSize > WEBCODECS_BACKLOG) this.gate.broken();
    if (this.wasm && this.wasm.queueSize > WASM_BACKLOG) this.gate.broken();
    const {action, reconfigure} = this.gate.accept(fr, wsSeq);
    if (action === "skip") return this.needKey();
    const id = Math.round(fr.seq * 1000);
    if (useWasm) {
      if (!this.wasm) return;
      if (reconfigure) this.wasm.reset();
      this.remember(id, fr);
      this.wasm.decode(fr.data, fr.width, fr.height, id);
      return;
    }
    try {
      if (!this.decoder || this.decoder.state === "closed") this.decoder = this.newDecoder();
      if (!this.decoder) return;
      if (reconfigure || this.decoder.state !== "configured") {
        if (this.decoder.state === "configured") this.decoder.reset();
        this.decoder.configure(cfg);
      }
      this.remember(id, fr);
      this.decoder.decode(new EncodedVideoChunk(chunkInit(fr)));
    } catch (e) {
      this.fail(`decoder: ${e instanceof Error ? e.message : String(e)}`);
      try {
        this.decoder?.close();
      } catch {
        /* already closed */
      }
      this.decoder = null;
    }
  }

  close() {
    this.closed = true;
    this.wasm?.close();
    try {
      this.decoder?.close();
    } catch {
      /* already closed */
    }
    this.meta.clear();
  }
}
