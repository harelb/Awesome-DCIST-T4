"""Per-machine network-interface byte counters from the ``tx_rx_monitor`` status notes.

tx_rx_monitor_dcist publishes its readings as ros_system_monitor notes, which
reach the base in both radio modes (status table relay)::

    [|IF|TX|RX|Δt|]:|enp118s0|284.74|33255.83|1.00|::|wlan0|ERR|ERR|ERR|:
      ;; TOT[|IF|TXTOT_B|RXTOT_B|SINCE_S|ADDR|]:|enp118s0|123|456|60.0|192.168.100.2|:

The first table is KiB moved during the last Δt seconds; the totals table
(newer monitors only) is bytes since the monitor started, seconds since then
and the interface's IPv4 address.
"""

from __future__ import annotations

import ipaddress
import re

TOTAL_SEPARATOR = ";; TOT"
_ROW = re.compile(r":\|([^|:]+)\|([^|:]*)\|([^|:]*)\|([^|:]*)\|(?:([^|:]*)\|)?:")


def _num(s: str) -> float | None:
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def is_tx_rx_node(nickname: str) -> bool:
    return "tx_rx" in (nickname or "")


def parse_tx_rx_notes(notes: str) -> dict:
    """``{iface: {tx_rate_Bps, rx_rate_Bps, dt_s, tx_total_B, rx_total_B, since_s,
    addr, error}}`` (total fields None when the monitor predates totals)."""
    rates, _, totals = (notes or "").partition(TOTAL_SEPARATOR)
    out: dict[str, dict] = {}
    for m in _ROW.finditer(rates):
        iface, tx, rx, dt = m.group(1), _num(m.group(2)), _num(m.group(3)), _num(m.group(4))
        ok = None not in (tx, rx, dt)
        out[iface] = {
            "tx_rate_Bps": tx * 1024.0 / dt if ok and dt > 0 else None,
            "rx_rate_Bps": rx * 1024.0 / dt if ok and dt > 0 else None,
            "dt_s": dt,
            "tx_total_B": None,
            "rx_total_B": None,
            "since_s": None,
            "addr": "",
            "error": not ok,
        }
    for m in _ROW.finditer(totals):
        iface = m.group(1)
        row = out.setdefault(
            iface,
            {"tx_rate_Bps": None, "rx_rate_Bps": None, "dt_s": None, "error": False},
        )
        tx, rx, since = _num(m.group(2)), _num(m.group(3)), _num(m.group(4))
        row["tx_total_B"] = int(tx) if tx is not None else None
        row["rx_total_B"] = int(rx) if rx is not None else None
        row["since_s"] = since
        row["addr"] = (m.group(5) or "").strip()
        row["error"] = row["error"] or tx is None
    return out


def _in_subnet(addr: str, subnet: str | None) -> bool:
    if not addr or not subnet:
        return False
    try:
        return ipaddress.ip_address(addr) in ipaddress.ip_network(subnet, strict=False)
    except ValueError:
        return False


def default_radio(
    ifaces: dict, topo: dict, network: str, machine: str, ip: str | None = None
) -> tuple[str | None, str | None]:
    """``(iface, network)`` most likely carrying the radio link of ``machine``.

    The interface holding the address the console reaches the machine on (or
    the machine's topology address on the active network); else the one in the
    active network's subnet; else one in a fleetlink network / any topology
    network.  ``(None, None)`` when no interface address matches.
    """
    networks = topo.get("networks") or {}
    addrs = ((topo.get("machines") or {}).get(machine) or {}).get("addresses") or {}
    for want in (ip, addrs.get(network)):
        for name, row in sorted(ifaces.items()):
            if want and row.get("addr") == want:
                net = next(
                    (n for n, v in networks.items() if _in_subnet(want, v.get("subnet"))),
                    network,
                )
                return name, net
    order = [network]
    order += list((topo.get("zenoh") or {}).get("fleetlink_networks") or [])
    order += list(networks)
    for net in dict.fromkeys(n for n in order if n):
        subnet = (networks.get(net) or {}).get("subnet")
        for name, row in sorted(ifaces.items()):
            if row.get("addr") == addrs.get(net) and row.get("addr"):
                return name, net
            if _in_subnet(row.get("addr", ""), subnet):
                return name, net
    return None, None


def interfaces_payload(fleet) -> dict:
    """``{network, machines: {name: {status, interfaces, radio_default,
    radio_network}}}`` from the fleet's latest ros_system_monitor table."""
    with fleet.lock:
        robots = dict((fleet.nodes or {}).get("robots") or {})
        updated = (fleet.nodes or {}).get("updated")
        topo = fleet.topo
        network = fleet.network
        ips = {n: m.get("ip") for n, m in fleet.runtime_config.items()}
    machines = {}
    for name, info in sorted(robots.items()):
        node = next(
            (n for n in info.get("nodes") or [] if is_tx_rx_node(n.get("nickname"))),
            None,
        )
        if node is None:
            continue
        ifaces = parse_tx_rx_notes(node.get("notes", ""))
        radio, radio_net = default_radio(ifaces, topo, network, name, ips.get(name))
        machines[name] = {
            "status": node.get("status"),
            "interfaces": ifaces,
            "has_totals": any(r.get("since_s") is not None for r in ifaces.values()),
            "radio_default": radio,
            "radio_network": radio_net,
        }
    return {"network": network, "updated": updated, "machines": machines}
