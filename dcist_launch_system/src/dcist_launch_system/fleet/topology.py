"""Topology editing and comment-preserving persistence.

``load_topology`` (fleet_helpers) returns plain dicts; edits are applied to
that dict and :func:`save_topology` merges it back into a ruamel round-trip
document so comments, key order and formatting in ``topology.yaml`` survive.
"""

from __future__ import annotations

import pathlib

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap

# Networks the editor exposes as explicit fields; others are preserved as-is.
EDITABLE_NETWORKS = ("mit_wifi", "silvus")


def ssh_user_for(topo: dict, role: str) -> str:
    ssh_conf = topo.get("ssh", {})
    if role == "base_station":
        return ssh_conf.get("base_station_user", "rrg")
    return ssh_conf.get("robot_user", "swarm")


def apply_machine_edit(
    topo: dict,
    runtime_config: dict,
    active_network: str,
    edit: dict,
    original_name: str | None = None,
) -> str:
    """Apply an editor result to ``topo`` (and ``runtime_config``); return the name.

    ``edit`` keys: name, role, platform, desc, mit_wifi, silvus.  Addresses on
    other networks and any extra keys (e.g. ``platform_type``) of the original
    entry are preserved.  Renaming removes the original entry.
    """
    name = edit["name"]
    role = edit["role"]
    platform = edit.get("platform", "")
    desc = edit.get("desc", "")
    machines = topo.setdefault("machines", {})
    original = dict(machines.get(original_name or name, {}))

    addrs = {
        net: ip
        for net, ip in original.get("addresses", {}).items()
        if net not in EDITABLE_NETWORKS
    }
    for net in EDITABLE_NETWORKS:
        if edit.get(net):
            addrs[net] = edit[net]

    entry = {
        k: v
        for k, v in original.items()
        if k not in ("role", "addresses", "platform_id", "desc")
    }
    entry = {"role": role, **entry, "addresses": addrs}
    if platform:
        entry["platform_id"] = platform
    if desc:
        entry["desc"] = desc

    if original_name and original_name != name:
        machines.pop(original_name, None)
        runtime_config.pop(original_name, None)
    machines[name] = entry

    if active_network in addrs:
        runtime_config[name] = {
            **runtime_config.get(name, {}),
            "name": name,
            "role": role,
            "ip": addrs[active_network],
            "user": ssh_user_for(topo, role),
            "desc": desc,
            "platform_id": platform or "",
        }
    else:
        runtime_config.pop(name, None)
    return name


def remove_machine(topo: dict, runtime_config: dict, name: str) -> None:
    topo.get("machines", {}).pop(name, None)
    runtime_config.pop(name, None)


def _merge(target, source):
    """Recursively make ``target`` (ruamel node) equal ``source`` (plain data)."""
    if isinstance(target, CommentedMap) and isinstance(source, dict):
        # ruamel stores a comment line *following* a key as that key's trailing
        # comment; when the key is deleted, hand it to the new last key.
        orphaned = None
        for key in [k for k in target if k not in source]:
            orphaned = target.ca.items.pop(key, None) or orphaned
            del target[key]
        if orphaned and target:
            last = list(target)[-1]
            if last not in target.ca.items and not isinstance(
                target[last], (dict, list)
            ):
                target.ca.items[last] = orphaned
        for key, value in source.items():
            if key in target and isinstance(value, (dict, list)):
                merged = _merge(target[key], value)
                if merged is not target[key]:
                    target[key] = merged
            elif key not in target or target[key] != value:
                target[key] = value
        return target
    if isinstance(target, list) and isinstance(source, list):
        if target == source:
            return target
        return source
    return source


def save_topology(path, topo: dict) -> None:
    """Write ``topo`` to ``path`` keeping the file's comments and layout."""
    path = pathlib.Path(path)
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)
    doc = yaml.load(path.read_text()) if path.exists() else None
    if not isinstance(doc, CommentedMap):
        doc = CommentedMap()
    _merge(doc, topo)
    with open(path, "w") as f:
        f.write("---\n")
        yaml.dump(doc, f)
