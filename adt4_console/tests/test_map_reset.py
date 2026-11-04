"""One-click map reset (API.md §6.6): what it would undo, where it reloads from,
the place / room features, and the SAM3 reload call. A fake Neo4j target (no DB)."""

import numpy as np
import pytest
from fastapi import HTTPException

from adt4_console.routes import maps


class FakeTarget:
    name, uri = "penn", "neo4j://x:7699"

    def __init__(self, source):
        self.source, self.calls = source, []

    def query(self, cypher, **params):
        self.calls.append((cypher, params))
        if "_GraphMetadata" in cypher and "RETURN" in cypher:
            return [{"p": self.source}] if self.source else []
        if "sam3_created" in cypher:
            return [{"c": 2}]
        if "GraphMutation" in cypher:
            return [{"c": 1}]
        return []

    def count_nodes(self):
        return 18535


def _run(tmp_path):
    backend = tmp_path / "fused" / "hydra" / "backend"
    backend.mkdir(parents=True)
    (backend / "dsg_with_mesh.json").write_text("{}")
    return backend


def test_status_says_what_a_reset_undoes(tmp_path):
    backend = _run(tmp_path)
    st = maps.reset_status(FakeTarget(str(backend / "dsg_with_mesh.json")))
    assert st["run"] == str(tmp_path / "fused") and st["added_objects"] == 2 and st["map_edits"] == 1
    assert st["nodes"] == 18535


def test_no_recorded_or_missing_source_is_a_409(tmp_path):
    for src in (None, str(tmp_path / "gone" / "dsg.json")):
        with pytest.raises(HTTPException) as e:
            maps.reset_status(FakeTarget(src))
        assert e.value.status_code == 409


def test_place_features_become_vector_properties(tmp_path):
    backend = _run(tmp_path)
    np.savez(backend / "semantic_features.npz", symbols=np.array(["P(12)", "R(3)"]), features=np.ones((2, 4), np.float32))
    t = FakeTarget(None)
    assert maps.load_semantic_features(t, tmp_path / "fused") == 2
    sets = [p for c, p in t.calls if "SET n.siglip2_emb" in c]
    assert [r["s"] for r in sets[0]["rows"]] == ["P12", "R3"]
    assert sum("CREATE VECTOR INDEX" in c for c, _ in t.calls) == 2
    assert maps.load_semantic_features(t, tmp_path / "nothing") == 0


def test_routes_refuse_an_unknown_target(settings):
    from .conftest import make_client

    client, _ = make_client(settings)
    with client:
        assert client.get("/api/maps/reset", params={"target": "nope"}).status_code == 404
        assert client.post("/api/maps/reset", json={"target": "nope"}).status_code == 404


def test_sam3_reload_posts_to_the_server(monkeypatch):
    from adt4_console import sam3_client

    seen = {}
    monkeypatch.setattr(sam3_client, "_post", lambda base, path, payload, t: seen.update(path=path, payload=payload) or {"before": 1, "after": 2})
    assert sam3_client.reload("http://127.0.0.1:8077") == {"before": 1, "after": 2}
    assert seen == {"path": "/reload", "payload": {}}
