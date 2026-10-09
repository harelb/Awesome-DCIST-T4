import pytest

from fleetlink.transport import LinkEmulator, VirtualClock, key_matches


def _collect(link):
    got = []
    link.base.subscribe("adt4/**", lambda k, d: got.append((link.clock(), k, len(d))))
    return got


def _run(link, until, dt=0.01):
    while link.clock() < until - 1e-9:
        link.clock.advance(dt)
        link.advance()


def test_key_matches():
    assert key_matches("adt4/**", "adt4/r/mesh/tile/1_2_0")
    assert key_matches("adt4/*/state", "adt4/euclid/state")
    assert not key_matches("adt4/*/state", "adt4/euclid/dsg/delta")
    assert key_matches("adt4/r/ctl/req", "adt4/r/ctl/req")


def test_bandwidth_and_latency():
    link = LinkEmulator(VirtualClock(), bandwidth=10_000, latency=0.15, mtu=1000)
    got = _collect(link)
    link.robot.put("adt4/r/x", bytes(5000), "C2")
    _run(link, 2.0)
    assert len(got) == 1
    assert got[0][0] == pytest.approx(0.5 + 0.15, abs=0.011)


def test_priority_preempts_at_fragment_level():
    link = LinkEmulator(VirtualClock(), bandwidth=10_000, latency=0.0, mtu=1000)
    got = _collect(link)
    link.robot.put("adt4/r/bulk", bytes(20_000), "C3")  # 2 s of airtime
    link.clock.advance(0.3)
    link.advance()
    link.robot.put("adt4/r/state", bytes(500), "C1")
    _run(link, 3.0)
    t_state = next(t for t, k, _ in got if k.endswith("state"))
    assert t_state < 0.5  # did not wait for the 2 s C3 message


def test_drop_vs_block_under_congestion():
    link = LinkEmulator(VirtualClock(), bandwidth=1000, latency=0.0, queue_bytes=4000)
    got = _collect(link)
    for i in range(10):
        # DROP class: overflow is dropped; BLOCK class: accepted, with backpressure
        link.robot.put(f"adt4/r/d{i}", bytes(1000), "C3")
        link.robot.put(f"adt4/r/b{i}", bytes(1000), "C2")
    assert link.robot.tx.stats.congestion_drops["C3"] == 6
    assert link.robot.tx.stats.congestion_drops["C2"] == 0
    assert not link.robot.can_send("C2")
    _run(link, 30.0, dt=0.1)
    assert sum(1 for _, k, _ in got if "/b" in k) == 10
    assert sum(1 for _, k, _ in got if "/d" in k) == 4
    assert link.robot.can_send("C2")


def test_loss_is_deterministic_and_reliable_classes_retransmit():
    def run(seed):
        link = LinkEmulator(
            VirtualClock(), bandwidth=None, latency=0.05, loss=0.2, seed=seed
        )
        got = _collect(link)
        for i in range(200):
            link.robot.put(f"adt4/r/c2/{i}", bytes(100), "C2")
            link.robot.put(f"adt4/r/c3/{i}", bytes(100), "C3")
        _run(link, 5.0)
        return got, link.robot.tx.stats

    a, sa = run(1)
    b, _ = run(1)
    assert a == b
    assert sum(1 for _, k, _ in a if "/c2/" in k) == 200  # reliable: all arrive
    n3 = sum(1 for _, k, _ in a if "/c3/" in k)
    assert 120 < n3 < 190 and sa.loss_drops["C3"] == 200 - n3
    assert sa.retransmits["C2"] > 0
