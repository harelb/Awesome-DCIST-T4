// tinyh264 ships plain JS: the emscripten module factory is all we use.
declare module "tinyh264/es/TinyH264" {
  export interface TinyH264Module {
    HEAPU8: Uint8Array;
    getValue(ptr: number, type: "i32" | "i8*"): number;
    _malloc(n: number): number;
    _free(p: number): void;
    _h264bsdAlloc(): number;
    _h264bsdFree(p: number): void;
    _h264bsdInit(storage: number, noOutputReordering: number): number;
    _h264bsdShutdown(storage: number): void;
    _h264bsdDecode(storage: number, data: number, len: number, pPicture: number, pWidth: number, pHeight: number): number;
  }
  const factory: (overrides?: Record<string, unknown>) => Promise<TinyH264Module>;
  export default factory;
}
