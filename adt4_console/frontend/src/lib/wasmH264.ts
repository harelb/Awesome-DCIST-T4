// Main-thread side of the WASM h264 decoder (h264.worker.ts). Loaded with a
// dynamic import only when the page has no WebCodecs, so desktop users never
// download it.

export interface WasmPicture {
  data: Uint8Array; // tight I420
  width: number;
  height: number;
  id: number;
  decodeMs: number; // time inside the worker
}

export class WasmH264Decoder {
  private worker: Worker;
  private inflight = 0;
  closed = false;

  constructor(
    private output: (p: WasmPicture) => void,
    private error: (message: string, id?: number) => void,
  ) {
    this.worker = new Worker(new URL("./h264.worker.ts", import.meta.url), {type: "module"});
    this.worker.onmessage = e => {
      const m = e.data as {type: string; id?: number; width?: number; height?: number; data?: ArrayBuffer; decodeMs?: number; message?: string};
      if (m.type === "frame" || m.type === "none" || (m.type === "error" && m.id !== undefined)) this.inflight = Math.max(0, this.inflight - 1);
      if (m.type === "frame") this.output({data: new Uint8Array(m.data!), width: m.width!, height: m.height!, id: m.id!, decodeMs: m.decodeMs ?? 0});
      else if (m.type === "error") this.error(m.message ?? "error", m.id);
    };
    this.worker.onerror = e => this.error(`worker: ${e.message}`);
  }

  /** Access units posted but not yet answered (the WebCodecs decodeQueueSize). */
  get queueSize() {
    return this.inflight;
  }

  /** Decode one Annex-B access unit; the picture is cropped to width x height. */
  decode(au: Uint8Array, width: number, height: number, id: number) {
    if (this.closed) return;
    const data = au.slice().buffer; // a private copy we can transfer
    this.inflight++;
    this.worker.postMessage({type: "decode", id, data, width, height}, [data]);
  }

  /** Forget the reference chain (before a new stream / after an error). */
  reset() {
    if (!this.closed) this.worker.postMessage({type: "reset"});
  }

  close() {
    this.closed = true;
    this.worker.terminate();
  }
}
