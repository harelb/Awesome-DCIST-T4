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
    # legacy model: C2 as zenoh BLOCK (accepted, with backpressure)
    link = LinkEmulator(
        VirtualClock(), bandwidth=1000, latency=0.0, queue_bytes=4000,
        block_classes=("C2",),
    )  # fmt: skip
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


def test_every_class_drops_by_default():
    """Like ZenohTransport: no class is BLOCK, overflow of C0 / C2 is dropped too."""
    link = LinkEmulator(VirtualClock(), bandwidth=1000, latency=0.0, queue_bytes=4000)
    for i in range(10):
        link.robot.put(f"adt4/r/b{i}", bytes(1000), "C2")
        link.robot.put(f"adt4/r/c{i}", bytes(1000), "C0")
    assert link.robot.tx.stats.congestion_drops["C2"] == 6
    assert link.robot.tx.stats.congestion_drops["C0"] == 6
    assert not link.robot.can_send("C2")  # backpressure=True: sees its queue
    blind = LinkEmulator(
        VirtualClock(), bandwidth=1000, queue_bytes=4000, backpressure=False
    )
    for i in range(10):
        blind.robot.put(f"adt4/r/b{i}", bytes(1000), "C2")
    assert blind.robot.can_send("C2")  # a zenoh sender cannot see the queue


def test_tcp_hol_big_message_delays_everything_behind_it():
    def first_state(tcp_hol):
        link = LinkEmulator(
            VirtualClock(), bandwidth=10_000, latency=0.0, mtu=1000, tcp_hol=tcp_hol
        )
        got = _collect(link)
        link.robot.put("adt4/r/bulk", bytes(20_000), "C3")  # 2 s of airtime
        link.clock.advance(0.3)
        link.advance()
        link.robot.put("adt4/r/state", bytes(500), "C1")
        _run(link, 3.0)
        order = [k for _, k, _ in got]
        return next(t for t, k, _ in got if k.endswith("state")), order

    t_prio, order_prio = first_state(False)
    t_hol, order_hol = first_state(True)
    assert t_prio < 0.5 and order_prio == ["adt4/r/state", "adt4/r/bulk"]
    assert t_hol >= 2.0 and order_hol == ["adt4/r/bulk", "adt4/r/state"]


def test_tcp_hol_queue_limit_is_shared():
    link = LinkEmulator(VirtualClock(), bandwidth=1000, queue_bytes=4000, tcp_hol=True)
    for i in range(4):
        link.robot.put(f"adt4/r/d{i}", bytes(1000), "C3")
    link.robot.put("adt4/r/state", bytes(100), "C1")  # the shared FIFO is full
    assert link.robot.tx.stats.congestion_drops["C1"] == 1
    assert not link.robot.can_send("C1")


def test_close_transport_drops_then_reopens():
    link = LinkEmulator(VirtualClock(), bandwidth=10_000, latency=0.1)
    got = _collect(link)
    link.robot.put("adt4/r/queued", bytes(5000), "C2")  # 0.5 s of airtime
    link.clock.advance(0.1)
    link.advance()
    link.close_transport(2.0)
    assert link.is_closed()
    link.robot.put("adt4/r/during", bytes(100), "C1")
    _run(link, 2.0)
    assert got == []
    st = link.robot.tx.stats
    assert st.closed_drops["C2"] == 1 and st.closed_drops["C1"] == 1
    _run(link, 2.2)
    assert not link.is_closed()
    link.robot.put("adt4/r/after", bytes(100), "C1")
    _run(link, 3.0)
    assert [k for _, k, _ in got] == ["adt4/r/after"]


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


def test_split_endpoints():
    from fleetlink.transport import split_endpoints

    assert split_endpoints("tcp/a:7450, tcp/b:7450,") == ["tcp/a:7450", "tcp/b:7450"]
    assert split_endpoints(["tcp/a:1", "tcp/b:2,tcp/c:3"]) == [
        "tcp/a:1",
        "tcp/b:2",
        "tcp/c:3",
    ]


def test_zenoh_publisher_cache_is_lru_capped():
    pytest.importorskip("zenoh")
    from fleetlink.transport import ZenohTransport

    class Pub:
        def __init__(self, key, kw):
            self.key, self.kw, self.puts, self.undeclared = key, kw, 0, False

        def put(self, data):
            self.puts += 1

        def undeclare(self):
            self.undeclared = True

    class Session:
        def __init__(self):
            self.pubs = []

        def declare_publisher(self, key, **kw):
            self.pubs.append(Pub(key, kw))
            return self.pubs[-1]

    import zenoh

    sess = Session()
    tr = ZenohTransport(session=sess, max_publishers=3)
    for i in range(4):
        tr.put(f"adt4/r/mesh/tile/{i}", b"x", "C3")
    tr.put("adt4/r/mesh/tile/1", b"x", "C3")  # cached: no new publisher
    assert len(sess.pubs) == 4 and tr.publisher_count() == 3
    assert sess.pubs[0].undeclared and not sess.pubs[1].undeclared  # LRU out
    tr.put("adt4/r/mesh/tile/5", b"x", "C3")
    assert sess.pubs[2].undeclared and not sess.pubs[1].undeclared  # 1 was used
    assert tr.undeclare("adt4/r/mesh/tile/1") == 1 and sess.pubs[1].undeclared
    assert all(
        p.kw["congestion_control"] == zenoh.CongestionControl.DROP for p in sess.pubs
    )
    tr.put("adt4/r/ctl/ack", b"x", "C0")
    assert sess.pubs[-1].kw["reliability"] == zenoh.Reliability.RELIABLE
    assert tr.can_send("C0")
