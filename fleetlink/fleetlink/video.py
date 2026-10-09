"""Live camera video (class C5, API.md §12).

The base turns a stream on with the C0 op ``video {camera, enable, fps, kbps,
width, codec}`` and repeats it as a keepalive; a stream with no ``enable``
request for ``keepalive_s`` (6 s) stops.  For every active stream the agent
samples the newest frame it keeps for that camera at ``fps``, scales it to
``width`` (aspect kept, even dimensions, never upscaled) and feeds it to an
encoder:

* ``h264`` / ``vp8``: the system ``ffmpeg`` (libx264 Annex-B with an AUD per
  access unit / libvpx in IVF) as a subprocess, raw RGB frames on stdin.  The
  output is split into frames here (:class:`AnnexBSplitter`, :class:`IvfParser`)
  and each frame is flagged ``key`` (IDR / VP8 key frame).
* ``jpeg``: one JPEG per frame (OpenCV), every frame a key frame.  Also the
  fallback when ffmpeg is missing or dies; the reason is reported in the state.

Frames go out on ``adt4/<r>/video/<camera>`` (zenoh DATA_LOW, DROP) through
their own token bucket on ``budgets.C5`` (:class:`C5Bucket`).  When the bucket
is empty, delta frames are dropped and key frames may borrow up to one burst;
after any drop the stream skips delta frames until the next key frame, because
a delta without its predecessors cannot be decoded.

Units: ``kbps`` = kilobits per second (encoder bitrate); budgets are bytes/s.
"""

from __future__ import annotations

import os
import select
import shutil
import struct
import subprocess
import threading
import time
from collections import deque
from typing import Callable

import numpy as np

from fleetlink import codec, media

FFMPEG = (
    os.environ.get("FLEETLINK_FFMPEG") or shutil.which("ffmpeg") or "/usr/bin/ffmpeg"
)

CODECS = ("h264", "vp8", "jpeg")
LIMITS = {"fps": (0.5, 15.0), "kbps": (50, 2000), "width": (64, 1920)}
DEFAULTS = {"fps": 5.0, "kbps": 300, "width": 640, "codec": "h264"}
KEEPALIVE_S = 6.0
MAX_STREAMS = 4
MIN_KBPS = LIMITS["kbps"][0]  # floor of the budget cap


class EncoderError(RuntimeError):
    pass


# ----------------------------------------------------------------------------
# Bitstream parsers (pure)
# ----------------------------------------------------------------------------


def _start_codes(buf: bytes, start: int = 0):
    """Yield (offset_of_start_code, offset_of_nal_header) for each 00 00 01."""
    i = start
    n = len(buf)
    while True:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0 or j + 3 >= n:
            return
        sc = j - 1 if j > 0 and buf[j - 1] == 0 else j
        yield sc, j + 3
        i = j + 3


def h264_nal_types(au: bytes) -> list[int]:
    return [au[h] & 0x1F for _, h in _start_codes(au)]


class AnnexBSplitter:
    """H.264 Annex-B byte stream → access units, split on access unit delimiters
    (NAL type 9; x264 ``aud=1``).  ``key`` = the AU holds an IDR slice (type 5)."""

    def __init__(self):
        self.buf = b""

    def feed(self, data: bytes) -> list[tuple[bytes, bool]]:
        self.buf += data
        auds = [sc for sc, h in _start_codes(self.buf) if self.buf[h] & 0x1F == 9]
        out = []
        for a, b in zip(auds, auds[1:]):
            out.append(self._au(self.buf[a:b]))
        if len(auds) >= 2:
            self.buf = self.buf[auds[-1] :]
        return out

    def has_complete_tail(self) -> bool:
        """The pending AU starts with an AUD and already contains a slice."""
        types = h264_nal_types(self.buf)
        return bool(types) and types[0] == 9 and any(t in (1, 5) for t in types)

    def flush(self) -> list[tuple[bytes, bool]]:
        """Emit the pending AU (stream end, or the encoder went quiet)."""
        if not self.buf or not any(t in (1, 5) for t in h264_nal_types(self.buf)):
            return []
        au, self.buf = self.buf, b""
        return [self._au(au)]

    @staticmethod
    def _au(au: bytes) -> tuple[bytes, bool]:
        return au, 5 in h264_nal_types(au)


class IvfParser:
    """IVF container (32-byte file header, then 12-byte frame headers) → VP8
    frames.  ``key`` = bit 0 of the VP8 frame tag is 0."""

    def __init__(self):
        self.buf = b""
        self.header: dict | None = None

    def feed(self, data: bytes) -> list[tuple[bytes, bool]]:
        self.buf += data
        out = []
        if self.header is None:
            if len(self.buf) < 32:
                return out
            if self.buf[:4] != b"DKIF":
                raise EncoderError("not an IVF stream")
            hlen = struct.unpack_from("<H", self.buf, 6)[0]
            fourcc = self.buf[8:12].decode("ascii", "replace")
            w, h = struct.unpack_from("<HH", self.buf, 12)
            self.header = {"fourcc": fourcc, "width": w, "height": h}
            self.buf = self.buf[max(32, hlen) :]
        while len(self.buf) >= 12:
            size = struct.unpack_from("<I", self.buf, 0)[0]
            if len(self.buf) < 12 + size:
                break
            frame = self.buf[12 : 12 + size]
            self.buf = self.buf[12 + size :]
            out.append((frame, bool(frame) and (frame[0] & 1) == 0))
        return out


# ----------------------------------------------------------------------------
# Encoders
# ----------------------------------------------------------------------------


def out_size(src_w: int, src_h: int, width: int) -> tuple[int, int]:
    """Requested width, never upscaled, aspect kept, both dimensions even."""
    w = min(int(width), int(src_w))
    w = max(2, w - (w % 2))
    h = max(2, int(round(src_h * w / src_w)))
    return w, h - (h % 2)


def ffmpeg_args(codec_: str, w: int, h: int, fps: float, kbps: int, path: str) -> list:
    g = str(max(1, int(round(2 * fps))))
    head = [path, "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", f"{fps:g}",
            "-i", "pipe:0", "-an", "-fps_mode", "passthrough", "-pix_fmt", "yuv420p",
            "-threads", "1", "-flush_packets", "1"]  # fmt: skip
    if codec_ == "h264":
        return head + [
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
            "-profile:v", "baseline", "-bf", "0", "-g", g, "-keyint_min", g,
            "-b:v", f"{kbps}k", "-maxrate", f"{kbps}k", "-bufsize", f"{kbps}k",
            "-x264-params", "aud=1:repeat-headers=1",
            "-f", "h264", "pipe:1",
        ]  # fmt: skip
    if codec_ == "vp8":
        return head + [
            "-c:v", "libvpx", "-deadline", "realtime", "-cpu-used", "8", "-g", g,
            "-b:v", f"{kbps}k", "-lag-in-frames", "0", "-auto-alt-ref", "0",
            "-f", "ivf", "pipe:1",
        ]  # fmt: skip
    raise ValueError(f"no ffmpeg codec {codec_!r}")


class FfmpegEncoder:
    """An ffmpeg subprocess: :meth:`feed` an RGB frame (non-blocking, newest
    pending frame wins), encoded frames come back through ``on_frame(data, key)``
    from the reader thread (or :meth:`poll` when no callback is given)."""

    def __init__(
        self,
        codec_: str,
        width: int,
        height: int,
        fps: float,
        kbps: int,
        on_frame: Callable[[bytes, bool], None] | None = None,
        path: str | None = None,
        flush_s: float = 0.02,
    ):
        self.codec = codec_
        self.width, self.height = int(width), int(height)
        self.fps, self.kbps = float(fps), int(kbps)
        self.path = path or FFMPEG
        self.flush_s = flush_s
        self.on_frame = on_frame
        self.out: deque = deque(maxlen=256)
        self.error: str | None = None
        self.frames_in = 0
        self.frames_out = 0
        self._closing = False
        self._pending: bytes | None = None
        self._cv = threading.Condition()
        self._stderr: deque = deque(maxlen=20)
        if not (os.path.isfile(self.path) and os.access(self.path, os.X_OK)):
            raise EncoderError(f"ffmpeg not found at {self.path}")
        try:
            self.proc = subprocess.Popen(
                ffmpeg_args(codec_, self.width, self.height, fps, kbps, self.path),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
                start_new_session=True,
            )
        except OSError as exc:
            raise EncoderError(f"cannot start ffmpeg: {exc}") from exc
        self._parser = AnnexBSplitter() if codec_ == "h264" else IvfParser()
        self._threads = [
            threading.Thread(target=fn, daemon=True, name=f"fleetlink-ffmpeg-{n}")
            for n, fn in (("w", self._writer), ("r", self._reader), ("e", self._errs))
        ]
        for t in self._threads:
            t.start()

    @property
    def alive(self) -> bool:
        return self.error is None and self.proc.poll() is None

    def feed(self, rgb: np.ndarray) -> None:
        if rgb.shape[:2] != (self.height, self.width):
            raise ValueError("frame size does not match the encoder")
        with self._cv:
            self._pending = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
            self._cv.notify()

    def poll(self) -> list[tuple[bytes, bool]]:
        out = []
        while self.out:
            out.append(self.out.popleft())
        return out

    def _emit(self, frames) -> None:
        for data, key in frames:
            self.frames_out += 1
            if self.on_frame is not None:
                try:
                    self.on_frame(data, key)
                except Exception:  # noqa: BLE001 - never kill the reader
                    pass
            else:
                self.out.append((data, key))

    def _writer(self) -> None:
        while True:
            with self._cv:
                while self._pending is None and not self._closing:
                    self._cv.wait(0.5)
                data, self._pending = self._pending, None
            if data is None:  # closing
                break
            try:
                self.proc.stdin.write(data)
                self.frames_in += 1
            except (BrokenPipeError, OSError, ValueError):
                if not self._closing:
                    self._fail("ffmpeg stdin closed")
                return
        try:
            self.proc.stdin.close()
        except Exception:  # noqa: BLE001
            pass

    def _reader(self) -> None:
        fd = self.proc.stdout.fileno()
        h264 = self.codec == "h264"
        while True:
            # an AU is only delimited by the next AUD; when x264 goes quiet with a
            # complete-looking AU pending, emit it instead of waiting a frame
            timeout = self.flush_s if h264 and self._parser.has_complete_tail() else 1.0
            ready, _, _ = select.select([fd], [], [], timeout)
            if not ready:
                if h264:
                    self._emit(self._parser.flush())
                continue
            try:
                chunk = os.read(fd, 1 << 16)
            except OSError:
                chunk = b""
            if not chunk:
                if h264:
                    self._emit(self._parser.flush())
                if not self._closing:
                    self.proc.wait(timeout=2.0)
                    self._fail(
                        f"ffmpeg exited ({self.proc.returncode})"
                        + (f": {self._stderr[-1]}" if self._stderr else "")
                    )
                return
            try:
                self._emit(self._parser.feed(chunk))
            except EncoderError as exc:
                self._fail(str(exc))
                return

    def _errs(self) -> None:
        for line in iter(self.proc.stderr.readline, b""):
            self._stderr.append(line.decode("utf-8", "replace").strip())

    def _fail(self, msg: str) -> None:
        if self.error is None:
            self.error = msg

    def close(self, wait: float = 2.0) -> None:
        """Stop feeding, let ffmpeg flush what it has, then reap it."""
        with self._cv:
            self._closing = True
            self._cv.notify_all()
        self._threads[0].join(timeout=wait)
        try:
            self.proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=wait)
        for t in self._threads[1:]:
            t.join(timeout=wait)


class JpegEncoder:
    """One JPEG per frame; quality follows the per-frame byte target."""

    codec = "jpeg"
    alive = True
    error = None

    def __init__(self, width, height, fps, kbps, on_frame=None, quality=70):
        self.width, self.height = int(width), int(height)
        self.fps, self.kbps = float(fps), int(kbps)
        self.on_frame = on_frame
        self.quality = int(quality)
        self.out: deque = deque(maxlen=64)
        self.frames_in = self.frames_out = 0

    def feed(self, rgb: np.ndarray) -> None:
        data, _fmt = media.encode_image(rgb, self.quality)
        self.frames_in += 1
        self.frames_out += 1
        target = self.kbps * 125.0 / max(self.fps, 1e-3)  # bytes per frame
        if len(data) > 1.15 * target:
            self.quality = max(10, self.quality - 5)
        elif len(data) < 0.75 * target:
            self.quality = min(90, self.quality + 2)
        if self.on_frame is not None:
            self.on_frame(data, True)
        else:
            self.out.append((data, True))

    def poll(self):
        out = list(self.out)
        self.out.clear()
        return out

    def close(self, wait: float = 0.0) -> None:
        pass


# ----------------------------------------------------------------------------
# C5 bucket
# ----------------------------------------------------------------------------


class C5Bucket:
    """Token bucket on the C5 budget shared by all video streams.

    ``admit(nbytes, key)``: delta frames need a positive balance; key frames may
    borrow up to one burst (``burst_s`` of budget) so a starved stream still gets
    key frames through.  Both go into debt by the frame they send."""

    def __init__(self, budget_fn: Callable[[], float | None], burst_s: float = 1.0):
        self.budget_fn = budget_fn
        self.burst_s = burst_s
        self.tokens: float | None = None
        self.t: float | None = None

    def _cap(self, b: float) -> float:
        return max(1.0, b * self.burst_s)

    def admit(self, nbytes: int, key: bool, now: float) -> bool:
        b = self.budget_fn()
        if b is None:
            return True
        cap = self._cap(b)
        if self.tokens is None:
            self.tokens = cap
        elif self.t is not None:
            self.tokens = min(cap, self.tokens + b * max(0.0, now - self.t))
        self.t = now
        floor = -cap if key else 0.0
        if self.tokens > floor:
            self.tokens -= nbytes
            return True
        return False


# ----------------------------------------------------------------------------
# Streams
# ----------------------------------------------------------------------------


def clamp_params(args: dict, current: dict | None = None) -> dict:
    """Validate / clamp ``{fps, kbps, width, codec}`` (missing → current/defaults)."""
    base = dict(DEFAULTS, **(current or {}))
    out = {}
    for k in ("fps", "kbps", "width"):
        v = args.get(k, base[k])
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise ValueError(f"bad {k} {v!r}") from None
        lo, hi = LIMITS[k]
        v = min(hi, max(lo, v))
        out[k] = round(v, 3) if k == "fps" else int(v)
    out["width"] -= out["width"] % 2
    c = str(args.get("codec", base["codec"])).lower()
    if c not in CODECS:
        raise ValueError(f"unknown codec {c!r} (use {', '.join(CODECS)})")
    out["codec"] = c
    return out


def to_rgb(frame: dict) -> np.ndarray:
    img = media.image_to_array(
        frame["encoding"], frame["width"], frame["height"], frame["step"], frame["data"]
    )
    if img.ndim == 2:
        img = np.repeat(img[:, :, None], 3, axis=2)
    return img


def scale(img: np.ndarray, w: int, h: int) -> np.ndarray:
    if img.shape[1] == w and img.shape[0] == h:
        return img
    if media._cv2 is not None:
        return media._cv2.resize(img, (w, h), interpolation=media._cv2.INTER_AREA)
    ys = np.minimum((np.arange(h) * img.shape[0] / h).astype(int), img.shape[0] - 1)
    xs = np.minimum((np.arange(w) * img.shape[1] / w).astype(int), img.shape[1] - 1)
    return np.ascontiguousarray(img[ys][:, xs])


FORCE_MIN_S = 0.5  # on-demand key frames (C0 video_keyframe), per stream
DROP_FORCE_MIN_S = 1.0  # after a budget drop: slower, a starved link would thrash


class _Stream:
    def __init__(self, camera: str, params: dict, now: float):
        self.camera = camera
        self.params = params
        self.last_req = now
        self.next_t = now
        self.enc = None
        self.enc_key: tuple | None = None  # (codec, w, h, fps, kbps)
        self.codec = params["codec"]  # actual codec (jpeg after a fallback)
        self.error: str | None = None
        self.fallback = False
        self.width = self.height = 0
        self.wait_key = False
        self.frames = 0
        self.keys = 0
        self.dropped = 0
        self.bytes = 0
        self.win_bytes = 0
        self.win_t = now
        self.sent_kbps = 0.0
        self.fed_t: deque = deque(maxlen=64)  # (t, stamp) per frame fed
        self.kbps_eff = int(params["kbps"])  # after the C5 budget cap
        self.forced = 0  # encoder restarts to get a key frame now
        self.last_force = -1e9

    def state(self) -> dict:
        out = {
            "codec": self.codec,
            "fps": self.params["fps"],
            "kbps": self.params["kbps"],
            "kbps_effective": self.kbps_eff,
            "width": self.width or self.params["width"],
            "height": self.height,
            "sent_kbps": round(self.sent_kbps, 1),
            "frames": self.frames,
            "dropped": self.dropped,
            "forced_keyframes": self.forced,
        }
        if self.error:
            out["error"] = self.error
        if self.codec != self.params["codec"]:
            out["requested_codec"] = self.params["codec"]
        return out


class VideoStreams:
    """All live video streams of one agent (pure logic + encoder subprocesses).

    ``get_frame(camera)`` returns the newest stored raw frame dict (or None);
    ``send(camera, payload)`` puts one C5 message.  :meth:`tick` must be called
    regularly (it expires streams and feeds encoders at their fps)."""

    def __init__(
        self,
        get_frame: Callable[[str], dict | None],
        send: Callable[[str, dict], int],
        budget_fn: Callable[[], float | None],
        clock: Callable[[], float] = time.time,
        cameras: Callable[[], list] = lambda: [],
        burst_s: float = 1.0,
        keepalive_s: float = KEEPALIVE_S,
        ffmpeg: str | None = None,
    ):
        self.get_frame = get_frame
        self.send = send
        self.cameras = cameras
        self.clock = clock
        self.keepalive_s = keepalive_s
        self.ffmpeg = ffmpeg
        self.bucket = C5Bucket(budget_fn, burst_s)
        self.streams: dict[str, _Stream] = {}
        self.seq: dict[str, int] = {}
        self._lock = threading.RLock()

    # -- control ---------------------------------------------------------

    def request(self, args: dict, now: float | None = None) -> dict:
        """The C0 ``video`` op.  Returns ``{camera, enabled, params, state}``."""
        now = self.clock() if now is None else now
        camera = str(args.get("camera") or "")
        names = list(self.cameras())
        if camera not in names:
            raise ValueError(f"unknown camera {camera!r} (have {names})")
        enable = bool(args.get("enable", True))
        with self._lock:
            st = self.streams.get(camera)
            if not enable:
                if st is not None:
                    self._stop(st)
                return {"camera": camera, "enabled": False}
            params = clamp_params(args, st.params if st else None)
            if st is None:
                if len(self.streams) >= MAX_STREAMS:
                    raise ValueError(f"at most {MAX_STREAMS} video streams")
                st = self.streams[camera] = _Stream(camera, params, now)
            elif params != st.params:
                st.params = params
                st.codec = params["codec"]
                st.error = None
                st.fallback = False
                self._close_encoder(st)  # rebuilt on the next frame
                st.next_t = now
            st.last_req = now
            return {
                "camera": camera,
                "enabled": True,
                "params": dict(params),
                "state": st.state(),
            }

    def force_keyframe(
        self, camera: str, now: float | None = None, min_gap: float = FORCE_MIN_S
    ) -> bool:
        """Get a key frame now instead of at the next GOP boundary (up to 2 s away).

        ffmpeg cannot be asked for an IDR over stdin, but a fresh encoder always
        starts with one, so the encoder is restarted (~50 ms) and the next frame
        is fed immediately.  At most once per ``min_gap`` seconds per stream."""
        now = self.clock() if now is None else now
        with self._lock:
            st = self.streams.get(camera)
            if st is None or now - st.last_force < min_gap:
                return False
            st.last_force = now
            st.forced += 1
            self._close_encoder(st)  # rebuilt (key frame first) on the next frame
            st.next_t = now
            return True

    def stop_all(self) -> None:
        with self._lock:
            for st in list(self.streams.values()):
                self._stop(st)

    close = stop_all

    def _stop(self, st: _Stream) -> None:
        self._close_encoder(st)
        self.streams.pop(st.camera, None)

    def _close_encoder(self, st: _Stream) -> None:
        enc, st.enc, st.enc_key = st.enc, None, None
        if enc is not None:
            enc.on_frame = None
            threading.Thread(target=enc.close, daemon=True).start()

    # -- state -----------------------------------------------------------

    def state(self) -> dict:
        with self._lock:
            return {c: st.state() for c, st in self.streams.items()}

    def next_due(self) -> float | None:
        with self._lock:
            return min((st.next_t for st in self.streams.values()), default=None)

    # -- tick ------------------------------------------------------------

    def tick(self, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        with self._lock:
            for st in list(self.streams.values()):
                if now - st.last_req > self.keepalive_s:
                    self._stop(st)
                    continue
                if now - st.win_t >= 1.0:
                    st.sent_kbps = st.win_bytes * 8 / 1000.0 / (now - st.win_t)
                    st.win_bytes, st.win_t = 0, now
                if st.enc is not None and not st.enc.alive:
                    self._fall_back(st, st.enc.error or "encoder stopped")
                if now < st.next_t:
                    continue
                period = 1.0 / st.params["fps"]
                st.next_t = (
                    st.next_t + period if now - st.next_t < period else now + period
                )
                frame = self.get_frame(st.camera)
                if frame is None:
                    continue
                try:
                    self._feed(st, frame, now)
                except Exception as exc:  # noqa: BLE001 - bad frame: report, go on
                    st.error = f"frame: {exc}"

    def effective_kbps(self, st: _Stream) -> int:
        """The requested bitrate, capped to fit the C5 budget (shared by all
        streams, 80% to leave room for key-frame peaks).  Encoding above the
        budget only makes the bucket drop frames — and every drop freezes the
        picture until a key frame — so the encoder is told the truth instead.
        Quantised to 25 kbit/s so small budget changes don't restart it."""
        want = int(st.params["kbps"])
        budget = self.bucket.budget_fn()
        if not budget or budget <= 0:
            return want
        share = 0.8 * float(budget) * 8 / 1000.0 / max(1, len(self.streams))
        cap = max(MIN_KBPS, int(share // 25) * 25)
        return min(want, cap)

    def _feed(self, st: _Stream, frame: dict, now: float) -> None:
        img = to_rgb(frame)
        w, h = out_size(img.shape[1], img.shape[0], st.params["width"])
        p = st.params
        st.kbps_eff = self.effective_kbps(st)
        want = (st.codec, w, h, p["fps"], st.kbps_eff)
        if st.enc is None or st.enc_key != want:
            self._close_encoder(st)
            st.enc = self._make_encoder(st, w, h)
            st.enc_key = want
            st.width, st.height = w, h
            st.wait_key = True  # a new encoder starts with a key frame
            st.fed_t.clear()
        st.fed_t.append((now, frame.get("stamp")))
        st.enc.feed(scale(img, w, h))

    def _force_later(self, camera: str, now: float) -> None:
        # called from an encoder's reader thread: closing that encoder there
        # would join itself, so hand the restart to a short-lived thread
        threading.Thread(
            target=self.force_keyframe,
            args=(camera, now, DROP_FORCE_MIN_S),
            daemon=True,
        ).start()

    def _make_encoder(self, st: _Stream, w: int, h: int):
        p = st.params

        def on_frame(data, key, st=st):
            self._on_encoded(st, data, key)

        if st.codec in ("h264", "vp8"):
            try:
                return FfmpegEncoder(
                    st.codec, w, h, p["fps"], st.kbps_eff, on_frame, self.ffmpeg
                )
            except EncoderError as exc:
                st.error = str(exc)
                st.fallback = True
                st.codec = "jpeg"
        return JpegEncoder(w, h, p["fps"], st.kbps_eff, on_frame)

    def _fall_back(self, st: _Stream, why: str) -> None:
        st.error = why
        st.fallback = True
        st.codec = "jpeg"
        self._close_encoder(st)

    def _on_encoded(self, st: _Stream, data: bytes, key: bool) -> None:
        now = self.clock()
        with self._lock:
            if self.streams.get(st.camera) is not st:
                return  # stopped / replaced
            t, stamp = st.fed_t.popleft() if st.fed_t else (now, None)
            if st.wait_key and not key:
                st.dropped += 1
                return
            if not self.bucket.admit(len(data), key, now):
                st.dropped += 1
                st.wait_key = True
                # recover with a fresh key frame instead of waiting out the GOP;
                # rarer than on request, since a starved budget would thrash
                self._force_later(st.camera, now)
                return
            st.wait_key = False
            seq = self.seq.get(st.camera, 0)
            self.seq[st.camera] = seq + 1
            payload = {
                "camera": st.camera,
                "codec": st.codec,
                "seq": seq,
                "t": t,
                "stamp": stamp,
                "key": bool(key),
                "width": st.width,
                "height": st.height,
                "fps": st.params["fps"],
                "data": bytes(data),
            }
            n = self.send(st.camera, payload)
            st.frames += 1
            st.keys += int(bool(key))
            st.bytes += n
            st.win_bytes += n


def make_sender(transport, robot: str, epoch_fn, clock, key_prefix: str):
    """``send(camera, payload)`` → one C5 put (``codec.pack(envelope)``); returns bytes."""

    def send(camera: str, payload: dict) -> int:
        stream = f"video/{camera}"
        env = codec.envelope(robot, stream, payload["seq"], epoch_fn(), payload)
        env["t"] = clock()
        data = codec.pack(env)
        transport.put(f"{key_prefix}/{stream}", data, "C5")
        return len(data)

    return send
