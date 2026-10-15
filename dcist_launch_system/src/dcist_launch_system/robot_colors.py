"""Per-robot display colors shared by RViz, the planner markers and the web console.

Source of truth is the optional ``color: "#rrggbb"`` field of each ``role: robot``
machine in topology.yaml. Robots without one get the next unused entry of PALETTE
(the same 20 colors as adt4_console/frontend/src/lib/palette.ts) by sorted name, so
every consumer derives the same mapping from the same file.

CLI:
    python -m dcist_launch_system.robot_colors [--topology PATH] [--name R]
        [--format json|ros|hex]

``ros`` prints ``name:#hex,name:#hex`` (the ``robot_colors`` ROS param format);
``hex`` prints a single color (requires ``--name``; empty if not a robot).
"""

import argparse
import json
import os
import pathlib
import re
import sys

import yaml

_DEFAULT_TOPOLOGY = (
    pathlib.Path(__file__).resolve().parents[3]
    / "network_diagnostics"
    / "config"
    / "topology.yaml"
)

# Keep in sync with adt4_console/frontend/src/lib/palette.ts PALETTE.
PALETTE = [
    "#e6194b", "#3cb44b", "#ffe119", "#4363d8", "#f58231",
    "#911eb4", "#42d4f4", "#f032e6", "#bfef45", "#fabed4",
    "#469990", "#dcbeff", "#9a6324", "#fffac8", "#800000",
    "#aaffc3", "#808000", "#ffd8b1", "#0075ff", "#a9a9a9",
]  # fmt: skip

_HEX_RE = re.compile(r"^#?([0-9a-fA-F]{6})$")


def normalize_hex(value):
    """Return ``#rrggbb`` (lowercase) or None if ``value`` is not a 6-digit hex color."""
    if not isinstance(value, str):
        return None
    m = _HEX_RE.match(value.strip())
    return f"#{m.group(1).lower()}" if m else None


def hex_to_rgb(value):
    """``#rrggbb`` -> (r, g, b) floats in [0, 1]."""
    h = normalize_hex(value)
    if h is None:
        raise ValueError(f"not a #rrggbb color: {value!r}")
    return tuple(int(h[i : i + 2], 16) / 255.0 for i in (1, 3, 5))


def hex_to_rgba(value, alpha=1.0):
    """``#rrggbb`` -> (r, g, b, a) floats in [0, 1]."""
    return (*hex_to_rgb(value), float(alpha))


def load_topology(path=None):
    path = pathlib.Path(path or os.environ.get("ADT4_TOPOLOGY") or _DEFAULT_TOPOLOGY)
    with open(path) as f:
        return yaml.safe_load(f) or {}


def robot_colors(topology):
    """Map every ``role: robot`` machine in ``topology`` to a ``#rrggbb`` color.

    Explicit (valid) ``color`` fields win; the rest take palette entries in sorted
    name order, skipping colors already claimed explicitly so robots stay distinct.
    """
    machines = (topology or {}).get("machines", {}) or {}
    robots = sorted(
        name
        for name, info in machines.items()
        if isinstance(info, dict) and info.get("role") == "robot"
    )
    colors = {}
    for name in robots:
        explicit = normalize_hex(machines[name].get("color"))
        if explicit:
            colors[name] = explicit
    used = set(colors.values())
    free = [c for c in PALETTE if c not in used]
    for idx, name in enumerate(n for n in robots if n not in colors):
        # Wrap around once the free palette runs out (> 20 robots).
        colors[name] = free[idx % len(free)] if free else PALETTE[idx % len(PALETTE)]
    return {name: colors[name] for name in robots}


def to_ros_string(colors):
    """``{name: hex}`` -> ``"name:#hex,name:#hex"``."""
    return ",".join(f"{name}:{color}" for name, color in colors.items())


def parse_ros_string(value):
    """Inverse of :func:`to_ros_string`; also accepts a list of ``name:#hex`` items.

    Malformed entries are skipped rather than raised on, so a bad param never
    takes down a node.
    """
    items = value if isinstance(value, (list, tuple)) else str(value or "").split(",")
    out = {}
    for item in items:
        name, sep, color = str(item).strip().partition(":")
        color = normalize_hex(color)
        if sep and name.strip() and color:
            out[name.strip()] = color
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--topology", help="topology.yaml (default: repo copy)")
    parser.add_argument("--name", help="only this robot")
    parser.add_argument("--format", choices=["json", "ros", "hex"], default="json")
    args = parser.parse_args(argv)

    colors = robot_colors(load_topology(args.topology))
    if args.name is not None:
        colors = {k: v for k, v in colors.items() if k == args.name}
    if args.format == "json":
        print(json.dumps(colors, indent=2))
    elif args.format == "ros":
        print(to_ros_string(colors))
    else:
        if args.name is None:
            parser.error("--format hex requires --name")
        print(colors.get(args.name, ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
