"""End-of-run bag sanity check.

``inspect_command`` runs on the machine (ROS sourced) and prints one JSON
object describing every segment of the run's bag there: finalized
(metadata.yaml), duration, per-topic message counts, the largest gap between
messages of the key topics, and the topic list the recorder was given.
``evaluate`` turns that into green / amber / red findings (pure; unit-tested).
"""

from __future__ import annotations

import json
import shlex

KEY_TOPIC_SUFFIXES = ("/odom", "/tf")  # missing or empty -> red
RATE_MIN_HZ = {"/odom": 5.0, "/tf": 5.0, "image_rect_color": 1.0, "/compressed": 1.0}
GAP_MAX_S = 2.0
DURATION_TOL = 0.10  # fraction of the run window
DURATION_TOL_S = 5.0
LOW_FREE_BYTES = 10 * 1024**3

_INSPECT_PY = r"""
import glob, json, os, sys
root, gap_topics_csv = sys.argv[1], sys.argv[2]
gap_suffixes = [s for s in gap_topics_csv.split(",") if s]
out = {"segments": [], "free_bytes": None}
try:
    st = os.statvfs(root)
    out["free_bytes"] = st.f_bavail * st.f_frsize
except OSError:
    pass
for d in sorted(p for p in glob.glob(os.path.join(root, "*")) if os.path.isdir(p)):
    seg = {"seg": os.path.basename(d), "metadata": False, "topics": {}, "gaps": {},
           "size": sum(os.path.getsize(f) for f in glob.glob(os.path.join(d, "*")) if os.path.isfile(f)),
           "files": len([f for f in glob.glob(os.path.join(d, "*")) if f.endswith((".mcap", ".db3"))])}
    lst = d + ".topics.txt"
    if os.path.isfile(lst):
        seg["expected"] = []
        for line in open(lst):
            t, _, c = line.partition("#")
            if t.strip():
                seg["expected"].append([t.strip(), c.strip().startswith("optional")])
    meta = os.path.join(d, "metadata.yaml")
    if os.path.isfile(meta):
        seg["metadata"] = True
        try:
            import yaml
            info = yaml.safe_load(open(meta))["rosbag2_bagfile_information"]
            seg["duration_s"] = info["duration"]["nanoseconds"] / 1e9
            seg["start_ns"] = info["starting_time"]["nanoseconds_since_epoch"]
            seg["message_count"] = info.get("message_count")
            seg["storage"] = info.get("storage_identifier")
            for t in info.get("topics_with_message_count", []):
                m = t["topic_metadata"]
                seg["topics"][m["name"]] = {"count": t["message_count"], "type": m["type"]}
        except Exception as e:
            seg["error"] = f"metadata: {e}"
        gap_topics = [t for t in seg["topics"] if any(t.endswith(s) for s in gap_suffixes) and seg["topics"][t]["count"] > 1]
        if gap_topics:
            try:
                import rosbag2_py
                r = rosbag2_py.SequentialReader()
                r.open(rosbag2_py.StorageOptions(uri=d, storage_id=seg.get("storage") or ""),
                       rosbag2_py.ConverterOptions("", ""))
                r.set_filter(rosbag2_py.StorageFilter(topics=gap_topics))
                last, gaps = {}, {}
                while r.has_next():
                    topic, _, t = r.read_next()
                    if topic in last:
                        g = (t - last[topic]) / 1e9
                        if g > gaps.get(topic, 0.0):
                            gaps[topic] = g
                    last[topic] = t
                seg["gaps"] = {k: round(v, 3) for k, v in gaps.items()}
            except Exception as e:
                seg["gap_error"] = str(e)[:200]
    out["segments"].append(seg)
print("ADT4_SANITY " + json.dumps(out))
"""


def inspect_command(bags_dir: str, gap_suffixes=("/odom",)) -> str:
    """``bags_dir`` relative to $HOME (``.../<run>/bags``); keeps only segments
    of this machine (the caller filters by name)."""
    return (
        'source "${ADT4_WS:-$HOME/dcist_ws}/install/setup.zsh" >/dev/null 2>&1; '
        f"python3 - \"$HOME/{bags_dir}\" {shlex.quote(','.join(gap_suffixes))} <<'ADT4_PY_EOF'\n"
        f"{_INSPECT_PY}\nADT4_PY_EOF\n"
    )


def parse_inspect(text: str) -> dict | None:
    for line in reversed((text or "").splitlines()):
        if line.startswith("ADT4_SANITY "):
            try:
                return json.loads(line[len("ADT4_SANITY "):])
            except ValueError:
                return None
    return None


def _is_key(topic: str) -> bool:
    return topic.endswith(KEY_TOPIC_SUFFIXES) and not topic.endswith("/tf_static")


def _min_rate(topic: str) -> float | None:
    for suffix, hz in RATE_MIN_HZ.items():
        if topic.endswith(suffix) or suffix in topic:
            return hz
    return None


def evaluate(machine: str, report: dict | None, segments: list[str], window_s: float | None,
             expected: list | None = None) -> dict:  # fmt: skip
    """{machine, level: ok|warn|err, findings: [{level, msg}], stats} for one machine."""
    findings: list[dict] = []

    def add(level, msg):
        findings.append({"level": level, "msg": msg})

    if report is None:
        add("err", "could not inspect the bag (machine unreachable or no python3/ROS)")
        return {"machine": machine, "level": "err", "findings": findings, "stats": {}}
    segs = [s for s in report.get("segments", []) if s["seg"] in segments] if segments else report.get("segments", [])
    if not segs:
        add("err", "no bag found")
        return {"machine": machine, "level": "err", "findings": findings, "stats": {}}
    counts: dict[str, int] = {}
    duration = 0.0
    size = 0
    for s in segs:
        size += s.get("size", 0)
        if not s.get("metadata"):
            add("err", f"{s['seg']}: not finalized (no metadata.yaml) — recorder killed before it closed the bag")
            continue
        if s.get("error"):
            add("warn", f"{s['seg']}: {s['error']}")
        duration += s.get("duration_s") or 0.0
        for t, v in (s.get("topics") or {}).items():
            counts[t] = counts.get(t, 0) + int(v.get("count") or 0)
        for t, g in (s.get("gaps") or {}).items():
            if g > GAP_MAX_S:
                add("warn", f"{s['seg']}: {t} has a {g:.1f} s gap")
        if s.get("gap_error"):
            add("dim", f"{s['seg']}: gap check skipped ({s['gap_error']})")
    exp = expected
    if exp is None:
        exp = next((s.get("expected") for s in segs if s.get("expected")), None) or []
    for t, optional in exp:
        n = counts.get(t, 0)
        if n == 0:
            if _is_key(t):
                add("err", f"{t}: no messages")
            elif not optional:
                add("warn", f"{t}: no messages")
    if duration > 0:
        for t, n in counts.items():
            hz = _min_rate(t)
            if hz and n > 0 and n / duration < hz:
                add("warn", f"{t}: {n / duration:.1f} Hz (expected >= {hz:g} Hz)")
    if window_s and duration > 0:
        diff = abs(duration - window_s)
        if diff > max(DURATION_TOL * window_s, DURATION_TOL_S):
            add("warn", f"bag covers {duration:.0f} s of a {window_s:.0f} s recording window")
    free = report.get("free_bytes")
    if free is not None and free < LOW_FREE_BYTES:
        add("warn", f"only {free / 1024**3:.1f} GB free on the disk")
    level = "err" if any(f["level"] == "err" for f in findings) else (
        "warn" if any(f["level"] == "warn" for f in findings) else "ok")  # fmt: skip
    return {
        "machine": machine,
        "level": level,
        "findings": findings,
        "stats": {
            "segments": len(segs),
            "duration_s": round(duration, 1),
            "size_bytes": size,
            "messages": sum(counts.values()),
            "topics_recorded": sum(1 for n in counts.values() if n > 0),
            "topics_expected": len(exp),
            "free_bytes": free,
        },
    }


def overall(results: list[dict]) -> str:
    levels = [r["level"] for r in results]
    return "err" if "err" in levels else "warn" if "warn" in levels else "ok" if levels else "none"
