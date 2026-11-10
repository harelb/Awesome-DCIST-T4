"""Manipulation approvals: the operator confirms a grasp pixel for Spot's Pick
(and the Drag skill, which reuses the same request/response).

``spot_executor_node`` publishes a ``ManipulationApprovalRequest`` (candidate
camera images, the detector's suggested pixel, optionally registered depth and
the walk-to preview geometry) on ``/<robot>/spot_executor_node/manipulation_request``
and waits for one ``ManipulationApprovalResponse`` on
``/<robot>/spot_executor_node/pick_confirmation``: approve the grasp at a pixel,
reject it, or walk toward a pixel first (the robot then captures new images and
sends a fresh request).  This module keeps the pending requests, serves their
images to the browser (``routes/manipulation.py``) and validates every answer
before the bridge publishes it; the request callback runs on the ROS executor
thread (``RosBridge._on_manipulation_request``).

The ROS protocol has no request id or cancellation message.  Local UUIDs, a
bounded lifetime and "only the newest request per robot is answerable" keep a
stale browser from answering twice; they cannot add wire-level correlation.
"""

from __future__ import annotations

import copy
import io
import logging
import threading
import time
import uuid
from types import SimpleNamespace

import numpy as np
from PIL import Image, ImageDraw

log = logging.getLogger(__name__)

REQUEST_TYPE = "nlu_interface_rviz/msg/ManipulationApprovalRequest"
RESPONSE_TYPE = "nlu_interface_rviz/msg/ManipulationApprovalResponse"
HUB_TOPIC = "manipulation/pending"
MAX_PIXELS = 25_000_000
# Depth shown to the operator: colour runs warm (near) to cool (far) over this range.
DEPTH_RANGE_M = (0.3, 6.0)
_DEPTH_STOPS = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
_DEPTH_COLORS = np.array(
    [[255, 48, 48], [255, 160, 0], [235, 235, 70], [70, 200, 255], [70, 80, 235]],
    dtype=np.float32,
)
HAND_CAMERA_WARNING = (
    "Hand camera: walk distances from this camera are unreliable. "
    "Pick walk-to points in a body camera."
)


def request_topic(robot: str) -> str:
    return f"/{robot}/spot_executor_node/manipulation_request"


def response_topic(robot: str) -> str:
    return f"/{robot}/spot_executor_node/pick_confirmation"


class ApprovalError(Exception):
    """A refused request/response (→ its HTTP status)."""

    def __init__(self, status_code: int, detail: str):
        self.status_code, self.detail = status_code, detail
        super().__init__(detail)


# ------------------------------------------------------------------ images
def image_png(message) -> bytes:
    """sensor_msgs/Image → PNG bytes, respecting encoding and row padding."""
    width, height = int(message.width), int(message.height)
    if width <= 0 or height <= 0 or width * height > MAX_PIXELS:
        raise ValueError("Image dimensions must be positive and at most 25 megapixels")
    modes = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "8UC3": ("RGB", "BGR", 3),
        "mono8": ("L", "L", 1),
        "8UC1": ("L", "L", 1),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "8UC4": ("RGBA", "BGRA", 4),
    }
    if message.encoding not in modes:
        raise ValueError(f"Unsupported image encoding {message.encoding!r}")
    mode, raw_mode, channels = modes[message.encoding]
    step = int(message.step) or width * channels
    data = bytes(message.data)
    if step < width * channels or len(data) < step * height:
        raise ValueError("Image data/stride is shorter than the declared dimensions")
    image = Image.frombytes(mode, (width, height), data, "raw", raw_mode, step, 1)
    out = io.BytesIO()
    image.save(out, format="PNG")
    return out.getvalue()


def compressed_image(message) -> tuple[bytes, str, int, int]:
    """Validate a sensor_msgs/CompressedImage → (bytes, media type, width, height)."""
    data = bytes(message.data)
    if not data:
        raise ValueError("Compressed image is empty")
    with Image.open(io.BytesIO(data)) as image:
        kind, (width, height) = image.format, image.size
    if kind not in {"JPEG", "PNG"}:
        raise ValueError(f"Unsupported compressed image format {kind!r}")
    if width <= 0 or height <= 0 or width * height > MAX_PIXELS:
        raise ValueError("Image dimensions must be positive and at most 25 megapixels")
    return data, f"image/{kind.lower()}", width, height


def decode_depth(message, scale):
    """16-bit single-channel PNG depth → (uint16 array, raw units per metre), or
    None when the entry is empty (no depth for that camera)."""
    data = bytes(getattr(message, "data", b"") or b"")
    if not data:
        return None
    if not float(scale) > 0:
        raise ValueError("Depth image has no positive depth scale")
    with Image.open(io.BytesIO(data)) as image:
        if image.format != "PNG" or image.mode not in {"I;16", "I;16L", "I;16B", "I"}:
            raise ValueError(
                f"Depth must be a 16-bit single-channel PNG, got {image.format} {image.mode}"
            )
        array = np.array(image)
    if array.ndim != 2 or array.size > MAX_PIXELS:
        raise ValueError("Depth image must be 2D and at most 25 megapixels")
    return np.clip(array, 0, 65535).astype(np.uint16), float(scale)


def depth_valid(depth):
    return (depth != 0) & (depth != 65535)


def depth_overlay_png(depth, scale) -> bytes:
    """Colourised RGBA depth; pixels without a measurement are fully transparent."""
    meters = depth.astype(np.float32) / scale
    low, high = DEPTH_RANGE_M
    t = np.clip((meters - low) / (high - low), 0.0, 1.0)
    rgba = np.zeros(depth.shape + (4,), dtype=np.uint8)
    for channel in range(3):
        rgba[..., channel] = np.interp(t, _DEPTH_STOPS, _DEPTH_COLORS[:, channel]).astype(np.uint8)
    rgba[..., 3] = np.where(depth_valid(depth), 255, 0).astype(np.uint8)
    out = io.BytesIO()
    Image.fromarray(rgba, "RGBA").save(out, format="PNG")
    return out.getvalue()


def depth_at(depth, scale, x, y, width, height, radius=2):
    """Median depth (m) near image pixel (x, y); the depth image may have a
    different resolution than the colour image."""
    rows, cols = depth.shape
    dx = min(cols - 1, max(0, int((x + 0.5) * cols / width)))
    dy = min(rows - 1, max(0, int((y + 0.5) * rows / height)))
    window = depth[max(0, dy - radius) : dy + radius + 1, max(0, dx - radius) : dx + radius + 1]
    values = window[depth_valid(window)]
    return None if values.size == 0 else round(float(np.median(values)) / scale, 3)


# ---------------------------------------------------------- walk-to preview
def quaternion_matrix(w, x, y, z):
    q = np.array([w, x, y, z], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 0.5:
        raise ValueError("Camera orientation is not a valid quaternion")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


EMPTY_GEOMETRY = {"cameras": {}, "ground": None, "robot": None, "standoff": 0.0, "max_distance": 0.0}


def walk_geometry(message, count) -> dict:
    """Parse the request's walk-to preview arrays (per image: 4 intrinsics and a
    7-float pose; one ground plane point+normal; the robot's xy+yaw); anything
    malformed is simply absent."""
    intrinsics = [float(v) for v in getattr(message, "camera_intrinsics", []) or []]
    poses = [float(v) for v in getattr(message, "camera_poses", []) or []]
    ground = [float(v) for v in getattr(message, "ground_plane", []) or []]
    robot = [float(v) for v in getattr(message, "robot_pose", []) or []]
    geometry = {
        "cameras": {},
        "ground": None,
        "robot": None,
        "standoff": float(getattr(message, "walk_to_standoff", 0.0) or 0.0),
        "max_distance": float(getattr(message, "walk_to_max_distance", 0.0) or 0.0),
    }
    for index in range(count):
        if len(intrinsics) < 4 * (index + 1) or len(poses) < 7 * (index + 1):
            break
        fx, fy, cx, cy = intrinsics[4 * index : 4 * index + 4]
        pose = poses[7 * index : 7 * index + 7]
        if fx <= 0 or fy <= 0 or not all(np.isfinite(intrinsics[4 * index : 4 * index + 4] + pose)):
            continue
        try:
            rotation = quaternion_matrix(*pose[3:])
        except ValueError:
            continue
        geometry["cameras"][index] = (fx, fy, cx, cy, np.array(pose[:3]), rotation)
    if len(ground) == 6 and np.all(np.isfinite(ground)) and np.linalg.norm(ground[3:]) > 0.5:
        geometry["ground"] = (np.array(ground[:3]), np.array(ground[3:]) / np.linalg.norm(ground[3:]))
    if len(robot) == 3 and np.all(np.isfinite(robot)):
        geometry["robot"] = tuple(robot)
    return geometry


def ground_estimate(geometry, index, x, y, min_range=0.2, max_range=15.0, min_move=0.05):
    """Reproduce the robot's walk-to plan for pixel (x, y): spot_tools
    ``walk_to_utils.pixel_to_ground_point`` + ``walk_to_geometry.plan_walk``.

    None when the request lacks the geometry; ``{'hit': False}`` when the ray
    misses the ground; else the horizontal robot→target distance, the planned
    move and whether it was capped."""
    camera = geometry["cameras"].get(index)
    if camera is None or geometry["ground"] is None or geometry["robot"] is None:
        return None
    fx, fy, cx, cy, origin, rotation = camera
    ray = np.array([(x - cx) / fx, (y - cy) / fy, 1.0])
    ray = rotation @ (ray / np.linalg.norm(ray))
    point, normal = geometry["ground"]
    denominator = float(ray @ normal)
    if abs(denominator) < 1e-6:
        return {"hit": False}
    t = float((point - origin) @ normal) / denominator
    if t < min_range or t > max_range:
        return {"hit": False}
    hit = origin + t * ray
    rx, ry, _ = geometry["robot"]
    distance = float(np.hypot(hit[0] - rx, hit[1] - ry))
    standoff, cap = geometry["standoff"], geometry["max_distance"]
    walk, capped = 0.0, False
    if distance - standoff >= min_move:
        walk = distance - standoff
        if cap > 0 and walk > cap:
            walk, capped = cap, True
    return {
        "hit": True,
        "distance_m": round(distance, 2),
        "walk_m": round(walk, 2),
        "capped": capped,
        "target": [round(float(v), 3) for v in hit],
    }


# ----------------------------------------------------------------- service
RESPONSE_DEFAULTS = {
    "approve": False,
    "image_index": 0,
    "image_x": 0,
    "image_y": 0,
    "walk_to": False,
    "has_stand_point": False,
    "stand_x": 0,
    "stand_y": 0,
}
OPEN = ("pending", "responding")


class ManipulationApprovalService:
    """Pending requests + answers.  ``publish(robot, payload) -> details`` sends
    one ``ManipulationApprovalResponse`` (see :func:`ros_response_publisher`);
    raising :class:`ApprovalError` leaves the request answerable, any other
    exception closes it (the message may have gone out).  ``available()`` /
    ``error()`` report the transport (the ROS bridge); simulated requests never
    publish and work without it."""

    def __init__(
        self,
        publish,
        hub=None,
        *,
        available=lambda: True,
        error=lambda: None,
        default_robot=lambda: "euclid",
        clock=time.time,
        request_ttl: float = 300.0,
    ):
        self.publish_hook = publish
        self.hub = hub
        self._available, self._error = available, error
        self._default_robot = default_robot
        self.clock, self.request_ttl = clock, request_ttl
        self._lock = threading.RLock()
        self._requests: dict[str, dict] = {}
        self._current: dict[str, str] = {}  # "<robot>" / "simulation:<robot>" -> request id

    @property
    def available(self) -> bool:
        return bool(self._available())

    @property
    def error(self):
        return self._error()

    # ------------------------------------------------------------- state
    def _expire_locked(self) -> None:
        now = self.clock()
        for request in self._requests.values():
            if request["status"] == "pending" and request["expires_at"] <= now:
                request["status"] = "expired"
                self._drop_media(request)
        # keep a bounded history of closed requests for diagnostics
        if len(self._requests) > 100:
            for key in list(self._requests):
                if len(self._requests) <= 100:
                    break
                if self._requests[key]["status"] not in OPEN:
                    del self._requests[key]

    @staticmethod
    def _drop_media(request) -> None:
        for key in ("_images", "_depth", "_depth_png", "_geometry"):
            request.get(key, {}).clear()

    @staticmethod
    def _public(request) -> dict:
        return {k: copy.deepcopy(v) for k, v in request.items() if not k.startswith("_")}

    def pending(self) -> dict:
        with self._lock:
            self._expire_locked()
            requests = [self._public(r) for r in self._requests.values() if r["status"] in OPEN]
            requests.sort(key=lambda r: (r["simulated"], r["received_at"]))
            return {"available": self.available, "error": self.error, "requests": requests}

    def _notify(self) -> None:
        if self.hub is None:
            return
        try:
            self.hub.publish(HUB_TOPIC, self.pending())
        except Exception:  # noqa: BLE001
            log.exception("publishing %s", HUB_TOPIC)

    # ----------------------------------------------------------- requests
    def receive_request(self, robot: str, message, *, simulated=False) -> dict:
        """A ManipulationApprovalRequest from ``robot`` (or a simulated one):
        decode its images, supersede the robot's previous open request."""
        declared = str(getattr(message, "robot_id", "") or "")
        if declared and declared != robot:
            raise ValueError("Request robot_id does not match its subscription")
        key = ("simulation:" if simulated else "") + robot
        identifier, now = uuid.uuid4().hex, self.clock()
        names = list(getattr(message, "image_source_names", []))
        raw_images = list(getattr(message, "images", []) or [])
        compressed = list(getattr(message, "compressed_images", []) or [])
        depth_messages = list(getattr(message, "depth_images", []) or [])
        depth_scales = list(getattr(message, "depth_scales", []) or [])
        base = f"/api/manipulation/requests/{identifier}"
        images, encoded, depth = [], {}, {}
        for index in range(len(raw_images) or len(compressed)):
            name = names[index] if index < len(names) else f"Image {index + 1}"
            item = {
                "index": index,
                "name": name,
                "source": name,
                "width": 0,
                "height": 0,
                "url": f"{base}/images/{index}",
            }
            try:
                if raw_images:
                    image = raw_images[index]
                    item["width"], item["height"] = int(image.width), int(image.height)
                    encoded[index] = (image_png(image), "image/png")
                else:
                    data, media, item["width"], item["height"] = compressed_image(compressed[index])
                    encoded[index] = (data, media)
            except (ValueError, OSError) as exc:
                item["error"] = str(exc)
            if index in encoded and index < len(depth_messages) and index < len(depth_scales):
                try:
                    decoded = decode_depth(depth_messages[index], depth_scales[index])
                except (ValueError, OSError) as exc:
                    decoded, item["depth_error"] = None, str(exc)
                if decoded is not None:
                    depth[index] = decoded
                    item["depth_url"] = f"{base}/depth/{index}.png"
            if index in encoded:
                item["point_query_url"] = f"{base}/point/{index}"
            images.append(item)
        geometry = walk_geometry(message, len(images))
        for item in images:
            item["ground_estimate"] = (
                item["index"] in geometry["cameras"]
                and geometry["ground"] is not None
                and geometry["robot"] is not None
            )
            if item["ground_estimate"] and str(item["source"]).startswith("hand"):
                # Measured on hamilton: the gripper camera sits high and often tilts
                # up, so the floor is a thin band near the horizon and its walk
                # estimates are not trustworthy.
                item["ground_estimate_warning"] = HAND_CAMERA_WARNING
        has_detection = bool(message.has_detection)
        image_index = int(message.detection_image_index)
        x, y = int(message.image_x), int(message.image_y)
        selection = None
        if (
            has_detection
            and 0 <= image_index < len(images)
            and image_index in encoded
            and 0 <= x < images[image_index]["width"]
            and 0 <= y < images[image_index]["height"]
        ):
            selection = {"image_index": image_index, "x": x, "y": y}
        request = {
            "id": identifier,
            "robot": robot,
            "received_at": now,
            "expires_at": now + self.request_ttl,
            "status": "pending",
            "simulated": bool(simulated),
            "images": images,
            "_images": encoded,
            "_depth": depth,
            "_depth_png": {},
            "depth_range_m": list(DEPTH_RANGE_M),
            "_geometry": geometry,
            "walk_to": {"standoff_m": geometry["standoff"], "max_distance_m": geometry["max_distance"]},
            "has_detection": has_detection,
            "detection_image_index": image_index,
            "image_x": x,
            "image_y": y,
            "selection": selection,
        }
        with self._lock:
            self._expire_locked()
            if simulated and self._live_open_locked():
                raise ApprovalError(
                    409, "A live manipulation request is pending; resolve it before opening a simulation"
                )
            old = self._current.get(key)
            if old in self._requests and self._requests[old]["status"] in OPEN:
                self._requests[old]["status"] = "superseded"
                self._drop_media(self._requests[old])
            self._current[key] = identifier
            self._requests[identifier] = request
            self._expire_locked()
        self._notify()
        return self._public(request)

    def _live_open_locked(self) -> bool:
        return any(not r["simulated"] and r["status"] in OPEN for r in self._requests.values())

    def get_image(self, identifier: str, index: int) -> tuple[bytes, str]:
        """(bytes, media type) of one candidate image."""
        with self._lock:
            request = self._requests.get(identifier)
            if request is None or index not in request["_images"]:
                raise ApprovalError(404, "Request image unavailable")
            return request["_images"][index]

    def _depth_locked(self, identifier, index):
        request = self._requests.get(identifier)
        if request is None or index not in request.get("_depth", {}):
            raise ApprovalError(404, "Depth unavailable for this image")
        return request, request["_depth"][index]

    def get_depth_overlay(self, identifier: str, index: int) -> bytes:
        with self._lock:
            request, (depth, scale) = self._depth_locked(identifier, index)
            cache = request["_depth_png"]
            if index not in cache:
                cache[index] = depth_overlay_png(depth, scale)
            return cache[index]

    def depth_value(self, identifier: str, index: int, x: int, y: int) -> dict:
        with self._lock:
            request, (depth, scale) = self._depth_locked(identifier, index)
            image = request["images"][index]
            if not (0 <= x < image["width"] and 0 <= y < image["height"]):
                raise ApprovalError(422, "Pixel is outside the image")
            return {
                "image_index": index,
                "x": x,
                "y": y,
                "meters": depth_at(depth, scale, x, y, image["width"], image["height"]),
            }

    def point_info(self, identifier: str, index: int, x: int, y: int) -> dict:
        """Measured depth (if any) and the robot's ground-plane walk estimate for one pixel."""
        with self._lock:
            request = self._requests.get(identifier)
            if request is None or index not in request["_images"]:
                raise ApprovalError(404, "Request image unavailable")
            image = request["images"][index]
            if not (0 <= x < image["width"] and 0 <= y < image["height"]):
                raise ApprovalError(422, "Pixel is outside the image")
            depth_m = None
            if index in request.get("_depth", {}):
                depth, scale = request["_depth"][index]
                depth_m = depth_at(depth, scale, x, y, image["width"], image["height"])
            return {
                "image_index": index,
                "x": x,
                "y": y,
                "has_depth": index in request.get("_depth", {}),
                "depth_m": depth_m,
                "ground": ground_estimate(request.get("_geometry") or EMPTY_GEOMETRY, index, x, y),
            }

    # ---------------------------------------------------------- responses
    def _active_locked(self, identifier, statuses=("pending",)) -> dict:
        self._expire_locked()
        request = self._requests.get(identifier)
        if request is None:
            raise ApprovalError(404, "Unknown manipulation request")
        key = ("simulation:" if request["simulated"] else "") + request["robot"]
        if request["status"] not in statuses or self._current.get(key) != identifier:
            raise ApprovalError(409, f"Request is {request['status']}; refresh the pending dialog")
        if request["expires_at"] <= self.clock():
            request["status"] = "expired"
            raise ApprovalError(409, "Manipulation request expired")
        return request

    def respond(
        self,
        identifier: str,
        approve: bool,
        *,
        image_index=None,
        image_x=None,
        image_y=None,
        walk_to=False,
        stand_x=None,
        stand_y=None,
    ) -> dict:
        """Publish one of three answers: approve the grasp, reject it, or walk
        closer first.  ``walk_to`` sends the selected pixel as the ground/object
        point to walk toward; the robot then captures new images and sends a
        fresh request.  ``stand_x``/``stand_y`` (walk_to only) is the optional
        ground pixel to stand on, facing the target.  Pixels default to the
        request's suggested detection."""
        with self._lock:
            request = self._active_locked(identifier)
            payload = dict(RESPONSE_DEFAULTS, approve=bool(approve))
            if approve and walk_to:
                raise ApprovalError(422, "A response is either an approval or a walk-to request, not both")
            if (stand_x is not None or stand_y is not None) and not walk_to:
                raise ApprovalError(422, "A stand point only applies to a walk-to request")
            if approve or walk_to:
                purpose = "Approve" if approve else "Walk-to"
                selection = request["selection"] or {}
                values = [image_index, image_x, image_y]
                if all(v is None for v in values):
                    values = [selection.get("image_index"), selection.get("x"), selection.get("y")]
                index, x, y = values
                if any(type(v) is not int for v in values):
                    raise ApprovalError(422, f"{purpose} requires an image and a valid pixel")
                if index < 0 or index >= len(request["images"]) or index not in request["_images"]:
                    raise ApprovalError(422, "Selected image is unavailable")
                image = request["images"][index]
                if not (0 <= x < image["width"] and 0 <= y < image["height"]):
                    raise ApprovalError(422, f"{purpose} pixel is outside the selected image")
                payload.update(image_index=index, image_x=x, image_y=y, walk_to=bool(walk_to))
                if walk_to and (stand_x is not None or stand_y is not None):
                    if type(stand_x) is not int or type(stand_y) is not int:
                        raise ApprovalError(422, "Stand point needs both pixel coordinates")
                    if not (0 <= stand_x < image["width"] and 0 <= stand_y < image["height"]):
                        raise ApprovalError(422, "Stand point is outside the selected image")
                    payload.update(has_stand_point=True, stand_x=stand_x, stand_y=stand_y)
            if request["simulated"]:
                request["status"] = "simulated"
                request["response"] = payload
                self._drop_media(request)
                result = {
                    "ok": True,
                    "status": "simulated",
                    "request_id": identifier,
                    "robot": request["robot"],
                    "response": payload,
                }
            else:
                if not self.available:
                    raise ApprovalError(503, self.error or "ROS unavailable: manipulation responses cannot be sent")
                request["status"] = "responding"
                result = self._publish_locked(request, payload)
        self._notify()
        return result

    def _publish_locked(self, request, payload) -> dict:
        identifier = request["id"]
        try:
            details = self.publish_hook(request["robot"], payload) or {}
        except ApprovalError:
            # nothing reached the robot: answerable again
            request["status"] = "pending"
            raise
        except Exception as exc:  # noqa: BLE001
            # the message may have gone out: close the request so it cannot be
            # approved twice; the robot's next request reopens the dialog
            request["status"] = "publish_failed"
            request["error"] = str(exc)
            self._drop_media(request)
            raise ApprovalError(
                503, f"Publication failed; request closed to prevent a duplicate approval: {exc}"
            ) from exc
        request["status"] = "published"
        request["response"] = payload
        self._drop_media(request)
        return {
            "ok": True,
            "status": "published",
            "request_id": identifier,
            "robot": request["robot"],
            "response": payload,
            **details,
        }

    # --------------------------------------------------------------- demo
    def create_demo(self, robot: str | None = None) -> dict:
        """A synthetic request (two cameras, depth, walk-to geometry) whose
        answers are recorded locally and never published."""
        robot = robot or self._default_robot()
        with self._lock:
            self._expire_locked()
            if self._live_open_locked():
                raise ApprovalError(
                    409, "A live manipulation request is pending; resolve it before opening a simulation"
                )
        images, depth_images = [], []
        rows = np.linspace(6.0, 0.6, 400, dtype=np.float32)[:, None].repeat(640, axis=1)
        rows[130:301, 180:421] = 1.5  # the simulated object stands 1.5 m away
        rows[:60, :] = 0.0  # no return from the sky: shown without depth
        for name, color in [("frontleft", "#34435b"), ("hand_color_image", "#493c35")]:
            image = Image.new("RGB", (640, 400), color)
            draw = ImageDraw.Draw(image)
            draw.rectangle((180, 130, 420, 300), fill="#c29b52", outline="white", width=3)
            draw.text((22, 22), "SIMULATION - no robot response", fill="white")
            draw.text((22, 48), name + ": select a grasp point or use the suggested detection", fill="white")
            images.append(
                SimpleNamespace(width=640, height=400, encoding="rgb8", step=1920, data=image.tobytes())
            )
            buffer = io.BytesIO()
            Image.fromarray((rows * 1000).astype(np.uint16)).save(buffer, format="PNG")
            depth_images.append(SimpleNamespace(data=buffer.getvalue()))
        message = SimpleNamespace(
            robot_id=robot,
            images=images,
            depth_images=depth_images,
            depth_scales=[1000.0, 1000.0],
            # a forward-looking camera 0.6 m up at the front of a robot at the origin
            camera_intrinsics=[400.0, 400.0, 320.0, 200.0] * 2,
            camera_poses=[0.3, 0.0, 0.6, 0.5, -0.5, 0.5, -0.5] * 2,
            ground_plane=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            robot_pose=[0.0, 0.0, 0.0],
            walk_to_standoff=1.0,
            walk_to_max_distance=5.0,
            image_source_names=["frontleft", "hand_color_image"],
            has_detection=True,
            detection_image_index=0,
            image_x=300,
            image_y=210,
        )
        return self.receive_request(robot, message, simulated=True)


# ---------------------------------------------------------------- ROS hook
def ros_response_publisher(ros):
    """``publish(robot, payload)`` for :class:`ManipulationApprovalService` that
    sends the response through :class:`RosBridge` (``publish_to_robot``).

    Router mode: the ROS match count must be non-zero, else nothing could have
    received it and the request stays answerable (503).  Fleetlink mode: the
    relay is always the local subscriber, so wait for the robot's ack instead;
    a missing ack is ambiguous (the relay keeps retrying), so that closes the
    request rather than inviting a duplicate approval."""

    def publish(robot: str, payload: dict) -> dict:
        out = ros.publish_to_robot(robot, response_topic(robot), RESPONSE_TYPE, payload)
        details = {k: out[k] for k in ("subscribers", "via", "delivered", "link") if k in out}
        if out.get("via") == "fleetlink":
            if out.get("error"):
                raise ApprovalError(503, f"Response not relayed to {robot}: {out['error']}")
            if not out.get("delivered"):
                raise RuntimeError(f"{robot} did not acknowledge the response in time")
        elif not out.get("subscribers"):
            raise ApprovalError(503, "No robot confirmation subscriber is currently discovered")
        return details

    return publish
