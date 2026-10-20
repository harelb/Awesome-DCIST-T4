"""Fleet state: topology, runtime machine config, reachability and status polling.

Thin wrapper over ``dcist_launch_system.fleet_helpers`` / ``fleet.*`` — the same
calls the Textual TUI makes, minus the UI.
"""

from __future__ import annotations

import logging
import pathlib
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

log = logging.getLogger(__name__)


def _helpers():
    from dcist_launch_system import fleet_helpers

    return fleet_helpers


@dataclass
class FleetDeps:
    """Side-effecting fleet_helpers functions, injectable for tests."""

    load_topology: Callable = None
    detect_network: Callable = None
    resolve_machines: Callable = None
    filter_reachable: Callable = None
    get_remote_status: Callable = None
    get_local_ips: Callable = None
    run_parallel: Callable = None
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        h = _helpers()
        for name in (
            "load_topology",
            "detect_network",
            "resolve_machines",
            "filter_reachable",
            "get_remote_status",
            "get_local_ips",
            "run_parallel",
        ):
            if getattr(self, name) is None:
                setattr(self, name, getattr(h, name))


class FleetService:
    def __init__(self, settings, hub=None, deps: FleetDeps | None = None):
        from dcist_launch_system.fleet import zenohd

        self.settings = settings
        self.hub = hub
        self.deps = deps or FleetDeps()
        self.topology_path = settings.topology or str(_helpers()._DEFAULT_TOPOLOGY)
        self.zenohd_state = zenohd.new_state()
        self.lock = threading.RLock()
        self.status: dict = {"updated": None, "machines": {}}
        self.nodes: dict = {"updated": None, "robots": {}}
        self._stop = threading.Event()
        self._poke = threading.Event()
        self._thread: threading.Thread | None = None
        self.reload()

    # ------------------------------------------------------------------ setup
    def reload(self) -> None:
        try:
            topo = self.deps.load_topology(self.topology_path) or {}
        except Exception as e:  # noqa: BLE001
            log.warning("topology %s not loaded: %s", self.topology_path, e)
            topo = {}
        network = self.settings.network
        if not network:
            try:
                network = self.deps.detect_network(topo) if topo else None
            except Exception as e:  # noqa: BLE001
                log.warning("network detection failed: %s", e)
        network = network or next(iter(topo.get("networks", {})), "") or ""
        with self.lock:
            self.topo = topo
            self.network = network
            self.local_ips = set(self.deps.get_local_ips())
            self.runtime_config = {
                m["name"]: dict(m)
                for m in (self.deps.resolve_machines(topo, network) if topo else [])
            }
            for name, m in self.runtime_config.items():
                ptype = topo.get("machines", {}).get(name, {}).get("platform_type")
                if ptype:
                    m["platform_type"] = ptype
                m["local"] = m["ip"] in self.local_ips

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._poll_loop, name="fleet-poll", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._poke.set()
        from dcist_launch_system.fleet import zenohd

        try:
            zenohd.cleanup(self.zenohd_state)
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------- machines
    @property
    def local_machine(self) -> dict | None:
        with self.lock:
            return next(
                (m for m in self.runtime_config.values() if m["ip"] in self.local_ips),
                None,
            )

    def machine(self, name: str) -> dict | None:
        with self.lock:
            m = self.runtime_config.get(name)
            return dict(m) if m else None

    def robots(self) -> list[str]:
        with self.lock:
            return sorted(
                n
                for n, m in self.topo.get("machines", {}).items()
                if m.get("role") == "robot"
            )

    def machines_payload(self) -> dict:
        with self.lock:
            machines = [
                {
                    "name": m["name"],
                    "role": m.get("role"),
                    "ip": m.get("ip"),
                    "user": m.get("user"),
                    "desc": m.get("desc", ""),
                    "platform_id": m.get("platform_id", ""),
                    "platform_type": m.get("platform_type", ""),
                    "online": m.get("online"),
                    "ping": m.get("ping"),
                    "ssh": m.get("ssh"),
                    "local": m.get("local", False),
                }
                for m in sorted(self.runtime_config.values(), key=lambda x: x["name"])
            ]
        return {"network": self.network, "machines": machines}

    def refresh_reachability(self) -> dict:
        with self.lock:
            machines = [dict(m) for m in self.runtime_config.values()]
        reachable, unreachable = self.deps.filter_reachable(machines)
        with self.lock:
            for m in reachable + unreachable:
                if m["name"] in self.runtime_config:
                    self.runtime_config[m["name"]].update(
                        {
                            "ping": m.get("ping", False),
                            "ssh": m.get("ssh", False),
                            "online": m.get("online", False),
                        }
                    )
        payload = self.machines_payload()
        self._publish("fleet/machines", payload)
        return payload

    # --------------------------------------------------------------- status
    def refresh_status(self) -> dict:
        with self.lock:
            machines = [
                dict(m) for m in self.runtime_config.values() if m.get("online")
            ]
            radios = self.topo.get("silvus_radios", {}).get("radios", {})
        silvus_ips = [r["mgmt_ip"] for r in radios.values() if "mgmt_ip" in r]

        def fetch(m):
            return self.deps.get_remote_status(
                m["user"],
                m["ip"],
                silvus_ips=silvus_ips or None,
                platform_type=m.get("platform_type"),
            )

        results = self.deps.run_parallel(fetch, machines) if machines else []
        status = {
            m["name"]: (r if isinstance(r, dict) else {"error": str(r)})
            for m, r in results
        }
        with self.lock:
            self.status = {"updated": time.time(), "machines": status}
            payload = dict(self.status)
        self._publish("fleet/status", payload)
        return payload

    def request_status_refresh(self) -> None:
        self._poke.set()

    def _poll_loop(self) -> None:
        try:
            self.refresh_reachability()
        except Exception:  # noqa: BLE001
            log.exception("initial reachability check failed")
        while not self._stop.is_set():
            try:
                self.refresh_status()
            except Exception:  # noqa: BLE001
                log.exception("status poll failed")
            self._poke.wait(self.settings.status_poll_s)
            self._poke.clear()

    # ------------------------------------------------------------ ROS nodes
    def update_nodes(self, robots: dict[str, list[dict]]) -> dict:
        """``robots``: {robot: [{nickname, node_name, status(int), notes, required}]}."""
        from dcist_launch_system.fleet_helpers import (
            _STATUS_NAMES,
            compute_robot_readiness,
        )

        out = {}
        for robot, nodes in robots.items():
            color, label = compute_robot_readiness(nodes)
            out[robot] = {
                "readiness": {"color": color, "label": label},
                "nodes": [
                    {
                        "nickname": n["nickname"],
                        "node_name": n.get("node_name", ""),
                        "status": _STATUS_NAMES.get(int(n["status"]), "STARTUP"),
                        "notes": n.get("notes", ""),
                        "required": bool(n.get("required", True)),
                    }
                    for n in nodes
                ],
            }
        with self.lock:
            self.nodes = {"updated": time.time(), "robots": out}
            payload = dict(self.nodes)
        self._publish("fleet/nodes", payload)
        return payload

    # ------------------------------------------------------------- topology
    def edit_machine(self, original_name: str, edit: dict) -> str:
        from dcist_launch_system.fleet.topology import apply_machine_edit

        with self.lock:
            name = apply_machine_edit(
                self.topo,
                self.runtime_config,
                self.network,
                edit,
                original_name=original_name,
            )
            if name in self.runtime_config:
                self.runtime_config[name]["local"] = (
                    self.runtime_config[name]["ip"] in self.local_ips
                )
        self._publish("fleet/machines", self.machines_payload())
        return name

    def remove_machine(self, name: str) -> None:
        from dcist_launch_system.fleet.topology import remove_machine

        with self.lock:
            remove_machine(self.topo, self.runtime_config, name)
        self._publish("fleet/machines", self.machines_payload())

    def save_topology(self) -> str:
        from dcist_launch_system.fleet.topology import save_topology

        with self.lock:
            save_topology(pathlib.Path(self.topology_path), self.topo)
        return self.topology_path

    def _publish(self, topic, data) -> None:
        if self.hub is not None:
            self.hub.publish(topic, data)
