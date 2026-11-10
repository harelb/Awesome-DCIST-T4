"""Manipulation approvals: request intake, image/depth serving, the exact wire
response, the HTTP routes and the bridge hook.  Nothing here touches ROS."""

import io
from types import SimpleNamespace as Obj

import numpy as np
import pytest
from PIL import Image

from adt4_console.manipulation_service import (
    HUB_TOPIC,
    ApprovalError,
    ManipulationApprovalService,
    ground_estimate,
    image_png,
    quaternion_matrix,
    ros_response_publisher,
    walk_geometry,
)
from adt4_console.ros_bridge import RosBridge
from tests.conftest import make_client


# ------------------------------------------------------------ fixtures
def image(encoding="rgb8", data=None, width=2, height=2, step=6):
    return Obj(
        width=width, height=height, encoding=encoding, step=step,
        data=data if data is not None else bytes([255, 0, 0] * 4),
    )  # fmt: skip


def request(*, detection=True, x=1, y=1, robot=""):
    return Obj(
        robot_id=robot, images=[image(), image()], image_source_names=["left", "hand"],
        has_detection=detection, detection_image_index=1, image_x=x, image_y=y,
    )  # fmt: skip


def jpeg_bytes(width=4, height=3, color=(200, 30, 30)):
    buffer = io.BytesIO()
    Image.new("RGB", (width, height), color).save(buffer, format="JPEG")
    return buffer.getvalue()


def depth_png(array):
    buffer = io.BytesIO()
    Image.fromarray(np.asarray(array, dtype=np.uint16)).save(buffer, format="PNG")
    return buffer.getvalue()


def compressed_request(depth=None, scales=(1000.0, 1000.0)):
    if depth is None:
        grid = np.full((3, 4), 1500, dtype=np.uint16)
        grid[0, :] = 0  # no measurement
        grid[2, 3] = 65535  # saturated / invalid
        depth = [Obj(data=depth_png(grid)), Obj(data=b"")]
    return Obj(
        robot_id="", images=[], image_source_names=["frontleft", "back"],
        compressed_images=[Obj(format="jpeg", data=jpeg_bytes()), Obj(format="jpeg", data=jpeg_bytes())],
        depth_images=depth, depth_scales=list(scales),
        has_detection=True, detection_image_index=0, image_x=1, image_y=2,
    )  # fmt: skip


def geometry_request(**overrides):
    """A camera 0.6 m up, 0.3 m ahead of a robot at the origin, looking along +x."""
    message = compressed_request()
    fields = dict(
        camera_intrinsics=[400.0, 400.0, 2.0, 1.0] * 2,
        camera_poses=[0.3, 0.0, 0.6, 0.5, -0.5, 0.5, -0.5] * 2,
        ground_plane=[0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        robot_pose=[0.0, 0.0, 0.0],
        walk_to_standoff=1.0,
        walk_to_max_distance=5.0,
    )
    fields.update(overrides)
    for key, value in fields.items():
        setattr(message, key, value)
    return message


def wire(**fields):
    """Exact ManipulationApprovalResponse payload: every field, defaults unless overridden."""
    base = {
        "approve": False, "image_index": 0, "image_x": 0, "image_y": 0,
        "walk_to": False, "has_stand_point": False, "stand_x": 0, "stand_y": 0,
    }  # fmt: skip
    return {**base, **fields}


class FakeHub:
    def __init__(self):
        self.frames = []

    def publish(self, topic, data):
        self.frames.append((topic, data))
        return len(self.frames)


def make_service(**over):
    now = [100.0]
    sent = []
    kw = dict(hub=FakeHub(), clock=lambda: now[0], request_ttl=30)
    kw.update(over)
    svc = ManipulationApprovalService(lambda robot, body: sent.append((robot, body)), **kw)
    svc.now, svc.sent = now, sent
    return svc


@pytest.fixture
def service():
    return make_service()


# ------------------------------------------------------------- service
def test_request_preserves_original_image_indices_and_suggested_detection(service):
    current = service.receive_request("euclid", request())
    assert current["selection"] == {"image_index": 1, "x": 1, "y": 1}
    assert current["images"][1]["name"] == "hand"
    assert current["images"][1]["url"].endswith("/images/1")
    data, media = service.get_image(current["id"], 1)
    assert media == "image/png" and "depth_url" not in current["images"][1]
    assert Image.open(io.BytesIO(data)).size == (2, 2)
    result = service.respond(current["id"], True)
    assert result["status"] == "published"
    assert service.sent == [("euclid", wire(approve=True, image_index=1, image_x=1, image_y=1))]
    assert service.pending()["requests"] == []


def test_pixel_override_and_reject_exact_wire_response(service):
    current = service.receive_request("euclid", request())
    service.respond(current["id"], True, image_index=0, image_x=0, image_y=1)
    assert service.sent[-1][1] == wire(approve=True, image_index=0, image_x=0, image_y=1)
    current = service.receive_request("euclid", request(detection=False))
    service.respond(current["id"], False)
    assert service.sent[-1][1] == wire()


def test_walk_to_publishes_target_and_optional_stand_point(service):
    current = service.receive_request("euclid", request())
    result = service.respond(current["id"], False, walk_to=True)
    assert result["status"] == "published"
    assert service.sent[-1][1] == wire(walk_to=True, image_index=1, image_x=1, image_y=1)
    assert service.pending()["requests"] == []
    # the robot answers a walk-to with a fresh request; the operator may then pick a stand point
    current = service.receive_request("euclid", request())
    service.respond(
        current["id"], False, walk_to=True, image_index=0, image_x=1, image_y=0, stand_x=0, stand_y=1
    )
    assert service.sent[-1][1] == wire(
        walk_to=True, image_index=0, image_x=1, image_y=0, has_stand_point=True, stand_x=0, stand_y=1
    )


@pytest.mark.parametrize(
    "fields",
    [
        dict(approve=True, walk_to=True),
        dict(approve=False, stand_x=0, stand_y=0),
        dict(approve=False, walk_to=True, stand_x=0),
        dict(approve=False, walk_to=True, stand_x=2, stand_y=0),
        dict(approve=False, walk_to=True, image_index=0, image_x=5, image_y=0),
    ],
)
def test_invalid_walk_to_requests_never_publish(service, fields):
    current = service.receive_request("euclid", request())
    with pytest.raises(ApprovalError) as failure:
        service.respond(current["id"], **fields)
    assert failure.value.status_code == 422
    assert not service.sent
    assert service.pending()["requests"][0]["id"] == current["id"]


@pytest.mark.parametrize(
    "fields",
    [
        dict(image_index=0, image_x=-1, image_y=0),
        dict(image_index=2, image_x=0, image_y=0),
        dict(image_index=0, image_x=2, image_y=0),
        dict(image_index=True, image_x=0, image_y=0),
        dict(image_index=0, image_x=1.2, image_y=0),
        dict(image_index=0),
    ],
)
def test_out_of_range_or_ambiguous_pixels_never_publish(service, fields):
    current = service.receive_request("euclid", request())
    with pytest.raises(ApprovalError) as error:
        service.respond(current["id"], True, **fields)
    assert error.value.status_code == 422
    assert not service.sent


def test_no_detection_requires_explicit_pixel_but_reject_still_works(service):
    current = service.receive_request("euclid", request(detection=False))
    with pytest.raises(ApprovalError):
        service.respond(current["id"], True)
    service.respond(current["id"], False)
    assert service.sent[-1][1]["approve"] is False


def test_superseded_expired_and_duplicate_browser_requests_do_not_publish_twice(service):
    old = service.receive_request("euclid", request())
    current = service.receive_request("euclid", request())
    with pytest.raises(ApprovalError) as error:
        service.respond(old["id"], True)
    assert error.value.status_code == 409
    service.respond(current["id"], True)
    with pytest.raises(ApprovalError):
        service.respond(current["id"], True)
    expired = service.receive_request("euclid", request())
    service.now[0] += 31
    with pytest.raises(ApprovalError) as error:
        service.respond(expired["id"], True)
    assert error.value.status_code == 409
    assert len(service.sent) == 1


def test_robots_are_independent_and_robot_id_must_match(service):
    first = service.receive_request("euclid", request())
    second = service.receive_request("hamilton", request(robot="hamilton"))
    assert [r["robot"] for r in service.pending()["requests"]] == ["euclid", "hamilton"]
    service.respond(second["id"], False)
    assert service.pending()["requests"][0]["id"] == first["id"]
    with pytest.raises(ValueError):
        service.receive_request("euclid", request(robot="hamilton"))


def test_pending_state_is_pushed_on_the_hub(service):
    current = service.receive_request("euclid", request())
    topic, state = service.hub.frames[-1]
    assert topic == HUB_TOPIC and state["requests"][0]["id"] == current["id"]
    assert "_images" not in state["requests"][0]
    service.respond(current["id"], False)
    assert service.hub.frames[-1][1]["requests"] == []


def test_transport_unavailable_blocks_live_answers_but_not_simulations():
    service = make_service(available=lambda: False, error=lambda: "ROS disabled (--no-ros)")
    assert service.pending() == {"available": False, "error": "ROS disabled (--no-ros)", "requests": []}
    live = service.receive_request("euclid", request())
    with pytest.raises(ApprovalError) as error:
        service.respond(live["id"], False)
    assert error.value.status_code == 503 and "--no-ros" in error.value.detail
    assert service.pending()["requests"][0]["status"] == "pending"
    service.now[0] += 31
    demo = service.create_demo()
    assert service.respond(demo["id"], True)["status"] == "simulated"
    assert not service.sent


def test_publish_failure_closes_the_request_but_a_refusal_keeps_it():
    def flaky(robot, payload):
        raise flaky.error

    service = make_service()
    service.publish_hook = flaky
    current = service.receive_request("euclid", request())
    flaky.error = ApprovalError(503, "nobody listening")
    with pytest.raises(ApprovalError) as refused:
        service.respond(current["id"], True)
    assert refused.value.detail == "nobody listening"
    assert service.pending()["requests"][0]["status"] == "pending"
    flaky.error = RuntimeError("relay timeout")
    with pytest.raises(ApprovalError) as failed:
        service.respond(current["id"], True)
    assert failed.value.status_code == 503 and "duplicate" in failed.value.detail
    assert service.pending()["requests"] == []
    assert service._requests[current["id"]]["status"] == "publish_failed"


@pytest.mark.parametrize("status", ["pending", "responding"])
def test_simulation_refuses_to_overlap_a_live_request(service, status):
    real = service.receive_request("euclid", request())
    service._requests[real["id"]]["status"] = status
    with pytest.raises(ApprovalError) as error:
        service.create_demo()
    assert error.value.status_code == 409
    assert [item["id"] for item in service.pending()["requests"]] == [real["id"]]
    assert not service.sent
    # rechecked at insertion too: a live request may arrive while demo images encode
    with pytest.raises(ApprovalError) as error:
        service.receive_request("euclid", request(), simulated=True)
    assert error.value.status_code == 409


def test_simulation_does_not_publish_and_expired_live_request_allows_demo(service):
    service.receive_request("euclid", request())
    service.now[0] += 31
    demo = service.create_demo()
    assert demo["simulated"] is True and demo["robot"] == "euclid"
    assert service.respond(demo["id"], True)["status"] == "simulated"
    assert not service.sent
    assert service.pending()["requests"] == []


# -------------------------------------------------------------- images
@pytest.mark.parametrize(
    "encoding, data, step",
    [
        ("rgb8", bytes([255, 0, 0, 0, 0]), 5),
        ("bgr8", bytes([0, 0, 255, 0, 0]), 5),
        ("8UC3", bytes([0, 0, 255, 0, 0]), 5),
        ("rgba8", bytes([255, 0, 0, 255, 0]), 5),
        ("bgra8", bytes([0, 0, 255, 255, 0]), 5),
        ("8UC4", bytes([0, 0, 255, 255, 0]), 5),
    ],
)
def test_image_encoding_and_padded_stride(encoding, data, step):
    raw = image(encoding, data, width=1, height=1, step=step)
    pixel = Image.open(io.BytesIO(image_png(raw))).convert("RGB").getpixel((0, 0))
    assert pixel == (255, 0, 0)


def test_mono_and_invalid_images():
    raw = image("mono8", bytes([42]), width=1, height=1, step=1)
    assert Image.open(io.BytesIO(image_png(raw))).getpixel((0, 0)) == 42
    with pytest.raises(ValueError):
        image_png(image("16UC1"))
    with pytest.raises(ValueError):
        image_png(image(data=b""))


def test_compressed_images_are_served_as_jpeg_with_registered_depth(service):
    current = service.receive_request("euclid", compressed_request())
    first, second = current["images"]
    assert (first["width"], first["height"]) == (4, 3)
    assert current["selection"] == {"image_index": 0, "x": 1, "y": 2}
    data, media = service.get_image(current["id"], 0)
    assert media == "image/jpeg" and data == jpeg_bytes()
    assert first["depth_url"].endswith("/depth/0.png") and first["point_query_url"].endswith("/point/0")
    assert "depth_url" not in second, "empty depth entry means no depth for that camera"
    assert current["depth_range_m"] == [0.3, 6.0]

    overlay = Image.open(io.BytesIO(service.get_depth_overlay(current["id"], 0)))
    assert overlay.mode == "RGBA" and overlay.size == (4, 3)
    assert overlay.getpixel((0, 0))[3] == 0, "missing depth is transparent"
    assert overlay.getpixel((3, 2))[3] == 0, "65535 is not a measurement"
    assert overlay.getpixel((1, 1))[3] == 255

    assert service.depth_value(current["id"], 0, 1, 1)["meters"] == 1.5
    assert service.depth_value(current["id"], 0, 1, 0)["meters"] == 1.5, "median ignores invalid neighbours"
    with pytest.raises(ApprovalError) as missing:
        service.depth_value(current["id"], 1, 0, 0)
    assert missing.value.status_code == 404
    with pytest.raises(ApprovalError) as outside:
        service.depth_value(current["id"], 0, 4, 0)
    assert outside.value.status_code == 422

    # responses are unchanged by the transport; answering drops all image and depth data
    service.respond(current["id"], True)
    assert service.sent[-1][1] == wire(approve=True, image_index=0, image_x=1, image_y=2)
    with pytest.raises(ApprovalError):
        service.get_depth_overlay(current["id"], 0)


def test_depth_at_a_different_resolution_maps_proportionally(service):
    grid = np.zeros((6, 8), dtype=np.uint16)
    grid[:, 4:] = 3000  # right half of the image is 3 m away
    current = service.receive_request(
        "euclid", compressed_request(depth=[Obj(data=depth_png(grid)), Obj(data=b"")])
    )
    assert service.depth_value(current["id"], 0, 3, 1)["meters"] == 3.0
    assert service.depth_value(current["id"], 0, 0, 1)["meters"] is None


@pytest.mark.parametrize(
    "depth,scales",
    [([Obj(data=b"not a png"), Obj(data=b"")], (1000.0, 1000.0)), (None, (0.0, 1000.0))],
)
def test_bad_depth_is_reported_without_losing_the_image(service, depth, scales):
    current = service.receive_request("euclid", compressed_request(depth=depth, scales=scales))
    first = current["images"][0]
    assert "depth_url" not in first and first["depth_error"]
    assert service.get_image(current["id"], 0)[1] == "image/jpeg"


def test_corrupt_compressed_image_marks_only_that_image(service):
    message = compressed_request()
    message.compressed_images[1] = Obj(format="jpeg", data=b"garbage")
    current = service.receive_request("euclid", message)
    assert "error" not in current["images"][0] and current["images"][1]["error"]
    with pytest.raises(ApprovalError):
        service.respond(current["id"], True, image_index=1, image_x=0, image_y=0)


def test_demo_request_carries_depth(service):
    current = service.create_demo()
    assert current["images"][0]["depth_url"]
    assert service.depth_value(current["id"], 0, 300, 210)["meters"] == 1.5
    assert service.depth_value(current["id"], 0, 10, 10)["meters"] is None


# ------------------------------------------------------- walk-to preview
def test_quaternion_convention_matches_camera_axes():
    rotation = quaternion_matrix(0.5, -0.5, 0.5, -0.5)
    assert np.allclose(rotation @ [0, 0, 1], [1, 0, 0]), "camera +z (forward) is vision +x"
    assert np.allclose(rotation @ [1, 0, 0], [0, -1, 0]), "camera +x (right) is vision -y"
    assert np.allclose(rotation @ [0, 1, 0], [0, 0, -1]), "camera +y (down) is vision -z"


def test_ground_estimate_reproduces_the_robot_plan():
    geometry = walk_geometry(geometry_request(), 2)
    # ~0.25 rad below the optical axis: the ray drops 0.6 m over 2.4 m -> hit 2.7 m from the robot
    estimate = ground_estimate(geometry, 0, 2, 101)
    assert estimate["hit"] and estimate["distance_m"] == 2.7 and estimate["walk_m"] == 1.7
    assert not estimate["capped"] and estimate["target"] == [2.7, 0.0, 0.0]
    # far (shallow) ray: 0.6 m over 9.6 m -> 9.9 m away, 8.9 m move capped to 5
    far = ground_estimate(geometry, 0, 2, 26)
    assert far["distance_m"] == 9.9 and far["walk_m"] == 5.0 and far["capped"]
    # within standoff + 5 cm: turn only
    near = ground_estimate(geometry, 0, 2, 401)
    assert near["distance_m"] < 1.05 and near["walk_m"] == 0.0
    # at or above the horizon the ray never meets the ground
    assert ground_estimate(geometry, 0, 2, 1) == {"hit": False}
    assert ground_estimate(geometry, 0, 2, 0) == {"hit": False}


def test_point_endpoint_combines_depth_and_ground_estimate(service):
    # wide-angle intrinsics for the 4x3 test images: pixel row 2 looks 0.5 down -> hit 1.5 m away
    current = service.receive_request(
        "euclid", geometry_request(camera_intrinsics=[2.0, 2.0, 2.0, 1.0] * 2)
    )
    assert current["images"][0]["ground_estimate"] is True
    assert current["walk_to"] == {"standoff_m": 1.0, "max_distance_m": 5.0}
    info = service.point_info(current["id"], 0, 1, 1)
    assert info["has_depth"] and info["depth_m"] == 1.5
    assert info["ground"] == {"hit": False}, "row 1 is the horizon"
    info = service.point_info(current["id"], 1, 2, 2)
    assert not info["has_depth"] and info["depth_m"] is None
    assert info["ground"] == {
        "hit": True, "distance_m": 1.5, "walk_m": 0.5, "capped": False, "target": [1.5, 0.0, 0.0],
    }  # fmt: skip


@pytest.mark.parametrize(
    "overrides",
    [
        dict(ground_plane=[]),
        dict(robot_pose=[1.0]),
        dict(camera_intrinsics=[0.0] * 8),
        dict(camera_poses=[0.0] * 14),
        dict(camera_intrinsics=[]),
    ],
)
def test_missing_or_invalid_geometry_disables_only_the_estimate(service, overrides):
    current = service.receive_request("euclid", geometry_request(**overrides))
    assert current["images"][0]["ground_estimate"] is False
    info = service.point_info(current["id"], 0, 1, 1)
    assert info["ground"] is None and info["depth_m"] == 1.5


def test_requests_without_geometry_still_answer_point_queries(service):
    current = service.receive_request("euclid", request())
    info = service.point_info(current["id"], 1, 1, 1)
    assert info == {"image_index": 1, "x": 1, "y": 1, "has_depth": False, "depth_m": None, "ground": None}


def test_demo_request_carries_ground_estimate(service):
    current = service.create_demo()
    info = service.point_info(current["id"], 0, 300, 300)
    assert info["ground"]["distance_m"] == 2.7 and info["ground"]["walk_m"] == 1.7


def test_hand_camera_estimates_are_flagged(service):
    message = geometry_request()
    message.image_source_names = ["frontleft_fisheye_image", "hand_color_image"]
    current = service.receive_request("euclid", message)
    body, hand = current["images"]
    assert body["ground_estimate"] and "ground_estimate_warning" not in body
    assert hand["ground_estimate"] and "unreliable" in hand["ground_estimate_warning"]
    # without geometry there is no estimate to warn about
    plain = compressed_request()
    plain.image_source_names = ["hand_color_image", "back"]
    assert "ground_estimate_warning" not in service.receive_request("euclid", plain)["images"][0]


# ----------------------------------------------------------------- HTTP
@pytest.fixture
def client(settings):
    client, services = make_client(settings)
    # the app's service answers through the (disabled) bridge; record instead
    services.manipulation = make_service(hub=services.hub)
    return client, services.manipulation


def test_http_api_validates_types_and_closes_only_current_request(client):
    client, service = client
    current = service.receive_request("euclid", request())
    assert client.get("/api/manipulation/pending").json()["requests"][0]["id"] == current["id"]
    path = f"/api/manipulation/requests/{current['id']}"
    assert client.get(path + "/images/1").headers["content-type"] == "image/png"
    assert client.get(path + "/images/7").status_code == 404
    assert client.post(path + "/respond", json={"approve": "true"}).status_code == 422
    assert client.post(path + "/respond", json={"approve": True, "image_x": 2.5}).status_code == 422
    assert not service.sent
    assert client.post(path + "/respond", json={"approve": True}).status_code == 200
    assert client.post(path + "/respond", json={"approve": False}).status_code == 409
    assert len(service.sent) == 1


def test_http_walk_to_body_is_validated_and_published(client):
    client, service = client
    current = service.receive_request("euclid", request())
    path = f"/api/manipulation/requests/{current['id']}"
    assert client.post(path + "/respond", json={"approve": True, "walk_to": True}).status_code == 422
    assert client.post(path + "/respond", json={"approve": False, "walk_to": "yes"}).status_code == 422
    assert client.post(path + "/respond", json={"approve": False, "stand_x": 0, "stand_y": 0}).status_code == 422
    assert not service.sent
    response = client.post(
        path + "/respond", json={"approve": False, "walk_to": True, "stand_x": 0, "stand_y": 1}
    )
    assert response.status_code == 200, response.text
    assert service.sent[-1][1] == wire(
        walk_to=True, image_index=1, image_x=1, image_y=1, has_stand_point=True, stand_x=0, stand_y=1
    )


def test_http_depth_routes(client):
    client, service = client
    current = service.receive_request("euclid", compressed_request())
    base = f"/api/manipulation/requests/{current['id']}"
    assert client.get(base + "/images/0").headers["content-type"] == "image/jpeg"
    assert client.get(base + "/depth/0.png").headers["content-type"] == "image/png"
    assert client.get(base + "/point/0", params={"x": 1, "y": 1}).json()["depth_m"] == 1.5
    assert client.get(base + "/depth/1.png").status_code == 404
    assert client.get(base + "/point/0", params={"x": "a", "y": 1}).status_code == 422
    assert client.get(base + "/point/0", params={"x": 9, "y": 1}).status_code == 422


def test_http_demo_reports_conflict_for_pending_live_request(client):
    client, service = client
    service.receive_request("euclid", request())
    result = client.post("/api/manipulation/demo")
    assert result.status_code == 409
    assert "live manipulation request" in result.json()["detail"]
    service.now[0] += 31
    demo = client.post("/api/manipulation/demo", params={"robot": "hamilton"}).json()
    assert demo["simulated"] and demo["robot"] == "hamilton"


def test_app_service_reports_the_bridge_state(settings):
    client, services = make_client(settings)
    state = client.get("/api/manipulation/pending").json()
    assert state["available"] is False and "--no-ros" in state["error"] and state["requests"] == []
    assert services.ros.manipulation is services.manipulation
    demo = client.post("/api/manipulation/demo").json()
    assert demo["simulated"] and demo["robot"] in services.fleet.robots()
    live = services.manipulation.receive_request("euclid", request())
    answer = client.post(f"/api/manipulation/requests/{live['id']}/respond", json={"approve": False})
    assert answer.status_code == 503 and "--no-ros" in answer.json()["detail"]
    assert client.post("/api/manipulation/demo").status_code == 409, "the live request is still open"


# --------------------------------------------------------------- bridge
def test_bridge_hands_requests_to_the_service_and_survives_bad_ones(settings):
    bridge = RosBridge(settings)
    bridge.manipulation = make_service()
    bridge._on_manipulation_request("euclid", request())
    assert bridge.manipulation.pending()["requests"][0]["robot"] == "euclid"
    bridge._on_manipulation_request("euclid", request(robot="hamilton"))  # logged, not raised
    assert len(bridge.manipulation.pending()["requests"]) == 1


class FakeRos:
    def __init__(self, out):
        self.out, self.calls = out, []

    def publish_to_robot(self, robot, topic, msg_type, fields, deliver_timeout=3.0):
        self.calls.append((robot, topic, msg_type, fields))
        return dict(self.out, topic=topic, msg=fields)


def test_router_mode_needs_a_matched_subscriber():
    ros = FakeRos({"ok": True, "subscribers": 1})
    publish = ros_response_publisher(ros)
    assert publish("euclid", wire(approve=True)) == {"subscribers": 1}
    _robot, topic, msg_type, fields = ros.calls[0]
    assert topic == "/euclid/spot_executor_node/pick_confirmation"
    assert msg_type == "nlu_interface_rviz/msg/ManipulationApprovalResponse"
    assert fields == wire(approve=True)
    with pytest.raises(ApprovalError) as error:
        ros_response_publisher(FakeRos({"ok": True, "subscribers": 0}))("euclid", wire())
    assert error.value.status_code == 503


def test_fleetlink_mode_waits_for_the_robot_ack():
    link = {"connected": True}
    delivered = ros_response_publisher(
        FakeRos({"ok": True, "subscribers": 1, "via": "fleetlink", "delivered": True, "link": link})
    )
    assert delivered("euclid", wire()) == {
        "subscribers": 1, "via": "fleetlink", "delivered": True, "link": link,
    }  # fmt: skip
    # not relayed: nothing left the base, so the request stays answerable
    with pytest.raises(ApprovalError) as refused:
        ros_response_publisher(
            FakeRos({"ok": True, "via": "fleetlink", "delivered": False, "subscribers": None,
                     "error": "topic not relayed"})
        )("euclid", wire())  # fmt: skip
    assert refused.value.status_code == 503 and "not relayed" in refused.value.detail
    # no ack: the relay keeps retrying, so the answer is closed, never repeated
    with pytest.raises(RuntimeError):
        ros_response_publisher(
            FakeRos({"ok": True, "via": "fleetlink", "delivered": False, "subscribers": None})
        )("euclid", wire())
