"""Auto-approver unit tests."""
from unittest.mock import MagicMock

from dcist_sim_ros.auto_approver import build_response


def test_approves_detection_passthrough():
    req = MagicMock(has_detection=True, detection_image_index=2, image_x=10, image_y=20)
    resp = build_response(req)
    assert resp.approve is True
    assert resp.image_index == 2
    assert (resp.image_x, resp.image_y) == (10, 20)


def test_rejects_when_no_detection():
    req = MagicMock(has_detection=False, detection_image_index=0, image_x=0, image_y=0)
    resp = build_response(req)
    assert resp.approve is False
