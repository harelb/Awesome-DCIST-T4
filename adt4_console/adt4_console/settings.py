"""Server configuration (built by ``cli.py`` or directly by tests)."""

from __future__ import annotations

import os
import pathlib
from dataclasses import dataclass, field

STATIC_DIR = pathlib.Path(__file__).resolve().parent / "static"


@dataclass
class Settings:
    host: str = "0.0.0.0"
    port: int = 8090
    token: str | None = None
    topology: str | None = None  # None -> fleet_helpers default topology.yaml
    network: str | None = None  # None -> detect_network()
    output_root: str = "~/adt4_output"
    neo4j: dict[str, str] = field(default_factory=dict)  # name -> uri
    prior_neo4j: str | None = None
    neo4j_user: str = field(
        default_factory=lambda: os.environ.get("HERACLES_NEO4J_USERNAME", "neo4j")
    )
    neo4j_password: str = field(
        default_factory=lambda: os.environ.get("HERACLES_NEO4J_PASSWORD", "")
    )
    maps: dict[str, str] = field(default_factory=dict)  # name -> run dir
    ros: bool = True
    fleetlink: bool = True
    zenoh_connect: str | None = None
    # live image-store mirror (fleetlink C6): None -> <output_root>/live, "" = off
    live_store_root: str | None = None
    robots: list[str] | None = None
    planner_ns: str | None = None  # omniplanner namespace (base station name)
    static_dir: pathlib.Path = STATIC_DIR

    # background work (tests switch these off)
    fleet_poll: bool = True
    status_poll_s: float = 10.0
    neo4j_poll_s: float = 2.0
    confirm_timeout_s: float = 300.0
    ros_domain_id: int | None = None  # None -> ROS_DOMAIN_ID env

    # chat
    llm: bool = True  # use an LLM when credentials are available
    # "auto": OpenAI if ADT4_OPENAI_API_KEY/OPENAI_API_KEY is set, else Anthropic
    llm_provider: str = "auto"
    llm_model: str | None = (
        None  # None -> provider default (gpt-4.1-mini / claude-opus-5-5)
    )
    llm_budget_usd: float = 10.0  # hard cap on cumulative chat spend (persisted)
    llm_usage_path: str = "~/.adt4_console/llm_usage.json"

    # SAM3 query server (agentic_navigation sam3_frontend/query_server.py):
    # chat's search_robot_images tool + the /api/sam3 commit/discard routes
    sam3_url: str = field(
        default_factory=lambda: os.environ.get(
            "SAM3_QUERY_SERVER_URL", "http://127.0.0.1:8077"
        )
    )
    sam3_timeout_s: float = 180.0

    @property
    def output_root_path(self) -> pathlib.Path:
        return pathlib.Path(os.path.expanduser(self.output_root))
