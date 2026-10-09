"""Silvus radio link-quality matrix and mesh routing, from ``get_silvus_radio_details``."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LinkCell:
    rssi_db: float | None = None  # measured RSSI
    linked: bool = False  # in the routing tree but no RSSI sample

    @property
    def quality(self) -> str:
        """good | fair | poor | linked | unknown"""
        if self.rssi_db is None:
            return "linked" if self.linked else "unknown"
        if self.rssi_db > -60:
            return "good"
        return "fair" if self.rssi_db > -80 else "poor"


def node_names(details: dict) -> dict[int, str]:
    """node_id -> radio name."""
    return {
        int(info["node_id"]): name
        for name, info in details.items()
        if info.get("node_id")
    }


def link_matrix(details: dict) -> dict[str, dict[str, LinkCell] | str]:
    """``{src: {dst: LinkCell}}``; an unreachable ``src`` maps to its error string."""
    names = sorted(details)
    matrix = {}
    for src in names:
        info = details[src]
        if info.get("error"):
            matrix[src] = info["error"]
            continue
        link_stats = info.get("link_stats")
        rssi_raw = info.get("rssi")
        routing_tree = [int(x) for x in (info.get("routing_tree") or [])]
        row = {}
        for dst in names:
            if dst == src:
                continue
            dst_nid = details[dst].get("node_id")
            val = None
            # link_stats is keyed by the neighbour's node_id string
            if isinstance(link_stats, dict) and dst_nid is not None:
                try:
                    val = float(
                        link_stats.get(str(dst_nid))
                        or link_stats.get(str(int(dst_nid)))
                    )
                except (TypeError, ValueError):
                    pass
            # Fallback: rssi field (single-neighbour case), in half-dB units
            if (
                val is None
                and isinstance(rssi_raw, list)
                and rssi_raw
                and len(names) == 2
            ):
                try:
                    val = float(rssi_raw[0]) / 2.0
                except (TypeError, ValueError):
                    pass
            linked = dst_nid is not None and int(dst_nid) in routing_tree
            row[dst] = LinkCell(val, linked)
        matrix[src] = row
    return matrix


def routing_summary(details: dict) -> dict[str, dict]:
    """Per reachable radio: node id, battery and routing tree resolved to names."""
    nid_to_name = node_names(details)
    return {
        name: {
            "node_id": info.get("node_id"),
            "battery": info.get("battery"),
            "routes": [
                nid_to_name.get(int(n), str(n)) for n in info.get("routing_tree") or []
            ],
        }
        for name, info in sorted(details.items())
        if not info.get("error")
    }
