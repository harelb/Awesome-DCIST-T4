#!/usr/bin/env python3
"""Record the topics of a ``recording/<name>_topics.txt`` file with ``ros2 bag record``.

Shared by ``run-adt4 -r`` (the "recording" tmux window) and the web console's
experiment runs (adt4_console/experiment/recorder.py).

Topic file format: one topic per line; ``${VAR}`` is expanded from the
environment (``ADT4_ROBOT_NAME``); blank lines and ``# comments`` are skipped; a
topic followed by ``# optional`` is recorded when present but a bag without it
still passes the console's sanity check.  A line using ``${ADT4_EACH_ROBOT}`` is
repeated once per robot in ``ADT4_FLEET_ROBOTS`` (comma separated).
"""

import os
import re

import click

EACH_ROBOT = re.compile(r"\$\{ADT4_EACH_ROBOT\}|\$ADT4_EACH_ROBOT\b")


def parse_topic_lines(lines, env=None, robots=None):
    """``[(topic, optional)]`` from topic-file lines (see the module doc)."""
    env = os.environ if env is None else env
    if robots is None:
        robots = [r for r in env.get("ADT4_FLEET_ROBOTS", "").split(",") if r]
    out, seen = [], set()
    for raw in lines:
        line, _, comment = raw.partition("#")
        line = line.strip()
        if not line:
            continue
        optional = comment.strip().lower().startswith("optional")
        expanded = [EACH_ROBOT.sub(r, line) for r in robots] if EACH_ROBOT.search(line) else [line]
        for t in expanded:
            t = _expand(t, env)
            if t and t not in seen and "$" not in t:
                seen.add(t)
                out.append((t, optional))
    return out


def _expand(text, env):
    def sub(m):
        return env.get(m.group(1) or m.group(2), m.group(0))

    return re.sub(r"\$\{(\w+)\}|\$(\w+)", sub, text)


def read_topic_file(path, env=None, robots=None):
    with open(path, "r") as fin:
        return parse_topic_lines(fin.read().splitlines(), env=env, robots=robots)


def record_command(output, topics, storage=None, max_bag_size=None, max_cache_size=None):
    """The ``ros2 bag record`` argv for ``topics`` (``[str]``).  A small
    ``max_cache_size`` (bytes) makes rosbag2 write to disk sooner (the default
    100 MB cache keeps a slow bag at 0 bytes for minutes)."""
    cmd = ["ros2", "bag", "record", "-o", str(output)]
    if storage:
        cmd += ["--storage", storage]
    if max_bag_size:
        cmd += ["--max-bag-size", str(int(max_bag_size))]
    if max_cache_size:
        cmd += ["--max-cache-size", str(int(max_cache_size))]
    return cmd + ["--topics", *topics]


@click.command
@click.argument("output", type=click.Path())
@click.argument("topic_file", type=click.Path(exists=True))
@click.option("--storage", default=None, help="rosbag2 storage plugin (mcap / sqlite3)")
@click.option("--max-bag-size", type=int, default=None, help="split the bag at this many bytes")
def main(output, topic_file, storage, max_bag_size):
    """Wrap rosbag record with topics from file"""
    topics = [t for t, _ in read_topic_file(topic_file)]
    cmd = record_command(output, topics, storage, max_bag_size)
    print(f"Running: '{' '.join(cmd)}'", flush=True)
    # exec, not subprocess.run: on Ctrl-C subprocess.run SIGKILLs the child
    # 0.25 s after the SIGINT, before rosbag2 has written metadata.yaml
    os.execvp(cmd[0], cmd)


if __name__ == "__main__":
    main()
