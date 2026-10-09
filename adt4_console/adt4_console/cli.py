"""``adt4-console`` command line (API.md §0)."""

from __future__ import annotations

import argparse
import logging
import os


def _pairs(values, flag) -> dict[str, str]:
    out = {}
    for v in values or []:
        if "=" not in v:
            raise SystemExit(f"{flag} expects NAME=VALUE, got {v!r}")
        k, val = v.split("=", 1)
        out[k.strip()] = val.strip()
    return out


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="adt4-console", description="ADT4 web console (base-station server)"
    )
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--token", default=os.environ.get("ADT4_CONSOLE_TOKEN"),
                   help="bearer token for /api (default: $ADT4_CONSOLE_TOKEN)")  # fmt: skip
    p.add_argument(
        "--topology", help="topology.yaml (default: network_diagnostics/config)"
    )
    p.add_argument("--network", help="active network (default: auto-detect)")
    p.add_argument("--output-root", default="~/adt4_output")
    p.add_argument("--neo4j", action="append", metavar="NAME=URI",
                   help="Neo4j graph source (creds: HERACLES_NEO4J_USERNAME/PASSWORD)")  # fmt: skip
    p.add_argument(
        "--prior-neo4j", metavar="NAME", help="which --neo4j target holds the prior"
    )
    p.add_argument("--map", action="append", metavar="NAME=RUN_DIR",
                   help="file graph source: run dir containing hydra/backend")  # fmt: skip
    p.add_argument("--no-ros", action="store_true")
    p.add_argument("--no-fleetlink", action="store_true")
    p.add_argument("--zenoh-connect", default="tcp/127.0.0.1:7447")
    p.add_argument(
        "--robots", help="comma-separated robots to plan for (default: topology)"
    )
    p.add_argument(
        "--planner-ns",
        help="namespace omniplanner runs in (default: $ADT4_ROBOT_NAME / local base station)",
    )
    p.add_argument(
        "--no-llm", action="store_true", help="always use the offline chat fallback"
    )
    p.add_argument("--log-level", default="info")
    return p.parse_args(argv)


def settings_from_args(args):
    from .settings import Settings

    neo4j = _pairs(args.neo4j, "--neo4j")
    if args.prior_neo4j and args.prior_neo4j not in neo4j:
        raise SystemExit(f"--prior-neo4j {args.prior_neo4j} is not a --neo4j target")
    return Settings(
        host=args.host,
        port=args.port,
        token=args.token or None,
        topology=args.topology,
        network=args.network,
        output_root=args.output_root,
        neo4j=neo4j,
        prior_neo4j=args.prior_neo4j,
        maps=_pairs(args.map, "--map"),
        ros=not args.no_ros,
        fleetlink=not args.no_fleetlink,
        zenoh_connect=args.zenoh_connect,
        robots=[r.strip() for r in args.robots.split(",") if r.strip()]
        if args.robots
        else None,
        llm=not args.no_llm,
        planner_ns=args.planner_ns,
    )


def main(argv=None):
    args = parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    settings = settings_from_args(args)

    import uvicorn

    from .app import create_app

    app = create_app(settings)
    uvicorn.run(app, host=settings.host, port=settings.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
