// Draws tight I420 pictures to a canvas: WebGL (three luminance textures and a
// BT.601 shader) or, without WebGL, a CPU conversion into ImageData.
import {i420ToRgba} from "./yuv";

const VS = `attribute vec2 p; varying vec2 uv;
void main() { uv = vec2((p.x + 1.0) * 0.5, (1.0 - p.y) * 0.5); gl_Position = vec4(p, 0.0, 1.0); }`;
// BT.601 limited range, the same matrix as i420ToRgba
const FS = `precision mediump float; varying vec2 uv; uniform sampler2D ty, tu, tv;
void main() {
  float y = 1.164 * (texture2D(ty, uv).r - 0.0627);
  float u = texture2D(tu, uv).r - 0.502, v = texture2D(tv, uv).r - 0.502;
  gl_FragColor = vec4(y + 1.596 * v, y - 0.392 * u - 0.813 * v, y + 2.017 * u, 1.0);
}`;

export class YuvRenderer {
  private gl: WebGLRenderingContext | null = null;
  private tex: WebGLTexture[] = [];
  private ctx2d: CanvasRenderingContext2D | null = null;
  private image: ImageData | null = null;
  readonly mode: "webgl" | "2d";

  constructor(private canvas: HTMLCanvasElement) {
    // a GPU-process restart loses every context; preventDefault lets the
    // browser restore ours, and we rebuild the program + textures then
    canvas.addEventListener("webglcontextlost", e => e.preventDefault());
    canvas.addEventListener("webglcontextrestored", () => {
      this.tex = [];
      this.gl = this.initGl();
    });
    this.gl = this.initGl();
    if (!this.gl) this.ctx2d = canvas.getContext("2d");
    this.mode = this.gl ? "webgl" : "2d";
  }

  private initGl(): WebGLRenderingContext | null {
    const gl = this.canvas.getContext("webgl", {antialias: false, depth: false, preserveDrawingBuffer: true});
    if (!gl) return null;
    const sh = (type: number, src: string) => {
      const s = gl.createShader(type)!;
      gl.shaderSource(s, src);
      gl.compileShader(s);
      return s;
    };
    const prog = gl.createProgram()!;
    gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS));
    gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) return null;
    gl.useProgram(prog);
    gl.bindBuffer(gl.ARRAY_BUFFER, gl.createBuffer());
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    const loc = gl.getAttribLocation(prog, "p");
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, 2, gl.FLOAT, false, 0, 0);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
    ["ty", "tu", "tv"].forEach((name, i) => {
      const t = gl.createTexture()!;
      gl.activeTexture(gl.TEXTURE0 + i);
      gl.bindTexture(gl.TEXTURE_2D, t);
      // NPOT textures in WebGL1: clamp + no mipmaps. NEAREST chroma matches
      // ffmpeg's own conversion (49 dB vs 30 dB for LINEAR on testsrc2); the
      // canvas is drawn at native size, CSS does any scaling.
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      gl.uniform1i(gl.getUniformLocation(prog, name), i);
      this.tex.push(t);
    });
    return gl;
  }

  draw(data: Uint8Array, w: number, h: number) {
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    const gl = this.gl;
    if (gl) {
      if (gl.isContextLost()) return; // until webglcontextrestored
      gl.viewport(0, 0, w, h);
      const cw = w / 2, ch = h / 2;
      const planes: [Uint8Array, number, number][] = [
        [data.subarray(0, w * h), w, h],
        [data.subarray(w * h, w * h + cw * ch), cw, ch],
        [data.subarray(w * h + cw * ch, w * h + 2 * cw * ch), cw, ch],
      ];
      planes.forEach(([p, pw, ph], i) => {
        gl.activeTexture(gl.TEXTURE0 + i);
        gl.bindTexture(gl.TEXTURE_2D, this.tex[i]);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.LUMINANCE, pw, ph, 0, gl.LUMINANCE, gl.UNSIGNED_BYTE, p);
      });
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
      return;
    }
    if (!this.ctx2d) return;
    if (!this.image || this.image.width !== w || this.image.height !== h) this.image = this.ctx2d.createImageData(w, h);
    i420ToRgba(data, w, h, this.image.data);
    this.ctx2d.putImageData(this.image, 0, 0);
  }
}
