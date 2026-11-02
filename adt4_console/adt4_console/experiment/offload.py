"""Offload run data (robot + base bags, the run folder) to an external disk
plugged into the base station, or to a network host over rsync, then verify
(size + md5 per file) and optionally delete the robots' copies.

Destination layout: ``<target>/<exp>/<run_id>/{run/, bags/<seg>/...}``.
"""

from __future__ import annotations

import getpass
import hashlib
import json
import os
import pathlib
import re
import shlex
import subprocess
import time

from .store import read_json, write_json

SKIP_MOUNTS = ("/", "/boot", "/boot/efi", "/home", "/var", "/tmp", "/usr", "/snap", "[SWAP]")
REMOVABLE_ROOTS = ("/media/", "/run/media/", "/mnt/")


# ----------------------------------------------------------------- targets
def parse_lsblk(text: str) -> list[dict]:
    """``lsblk -J -b -o NAME,LABEL,SIZE,FSAVAIL,FSTYPE,MOUNTPOINT(S),RM,HOTPLUG,TRAN``
    -> mounted external filesystems ``[{id, kind: disk, path, label, size_bytes,
    free_bytes, fstype, device, transport}]``."""
    try:
        data = json.loads(text)
    except ValueError:
        return []
    out = []

    def walk(dev, parent=None):
        tran = dev.get("tran") or (parent or {}).get("tran")
        removable = _truthy(dev.get("rm")) or _truthy(dev.get("hotplug")) or \
            _truthy((parent or {}).get("rm")) or _truthy((parent or {}).get("hotplug")) or tran == "usb"  # fmt: skip
        mounts = dev.get("mountpoints") or ([dev.get("mountpoint")] if dev.get("mountpoint") else [])
        for mp in [m for m in mounts if m]:
            if mp in SKIP_MOUNTS or mp.startswith("/snap/") or mp.startswith("/boot"):
                continue
            if not (removable or mp.startswith(REMOVABLE_ROOTS)):
                continue
            out.append({
                "id": f"disk:{mp}", "kind": "disk", "path": mp,
                "label": dev.get("label") or os.path.basename(mp) or dev.get("name"),
                "size_bytes": _int(dev.get("size")), "free_bytes": _int(dev.get("fsavail")),
                "fstype": dev.get("fstype"), "device": f"/dev/{dev.get('name')}", "transport": tran,
            })  # fmt: skip
        for ch in dev.get("children") or []:
            walk(ch, dev)

    for dev in data.get("blockdevices") or []:
        walk(dev)
    return out


def _truthy(v) -> bool:
    return v in (True, 1, "1", "true")


def _int(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def detect_disks(run=subprocess.run) -> list[dict]:
    cols = "NAME,LABEL,SIZE,FSAVAIL,FSTYPE,MOUNTPOINTS,RM,HOTPLUG,TRAN"
    for c in (cols, cols.replace("MOUNTPOINTS", "MOUNTPOINT")):  # util-linux < 2.37 has no MOUNTPOINTS
        try:
            r = run(["lsblk", "-J", "-b", "-o", c], capture_output=True, text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return []
        if r.returncode == 0:
            return parse_lsblk(r.stdout)
    return []


NET_RE = re.compile(r"^(?:(?P<user>[^@\s]+)@)?(?P<host>[^:\s]+):(?P<path>\S+)$")


def parse_network_target(spec: str) -> dict:
    m = NET_RE.match(spec.strip())
    if not m:
        raise ValueError("network target is [user@]host:/path")
    user = m.group("user") or getpass.getuser()
    return {"id": f"net:{user}@{m.group('host')}:{m.group('path')}", "kind": "network",
            "user": user, "host": m.group("host"), "path": m.group("path"),
            "label": f"{user}@{m.group('host')}:{m.group('path')}"}  # fmt: skip


class Targets:
    """Saved network targets (``offload_targets.json``) + detected disks."""

    def __init__(self, path: pathlib.Path | str | None, detect=detect_disks):
        self.path = pathlib.Path(os.path.expanduser(str(path))) if path else None
        self.detect = detect
        self._mem: list[dict] = []

    def network(self) -> list[dict]:
        if self.path is None:
            return list(self._mem)
        return list(read_json(self.path, []) or [])

    def add_network(self, spec: str, name: str = "") -> dict:
        t = parse_network_target(spec)
        if name:
            t["label"] = name
        nets = [n for n in self.network() if n["id"] != t["id"]] + [t]
        self._save(nets)
        return t

    def remove(self, target_id: str) -> None:
        self._save([n for n in self.network() if n["id"] != target_id])

    def _save(self, nets):
        if self.path is None:
            self._mem = nets
        else:
            write_json(self.path, nets)

    def all(self) -> list[dict]:
        disks = []
        for d in self.detect():
            try:
                st = os.statvfs(d["path"])
                d["free_bytes"] = st.f_bavail * st.f_frsize
                d["writable"] = os.access(d["path"], os.W_OK)
            except OSError:
                d["writable"] = False
            disks.append(d)
        return disks + self.network()

    def get(self, target_id: str) -> dict:
        for t in self.all():
            if t["id"] == target_id:
                return t
        if target_id.startswith("disk:"):
            p = target_id[5:]
            if os.path.isdir(p):  # any directory on the base (e.g. a mounted NAS)
                return {"id": target_id, "kind": "disk", "path": p, "label": p, "writable": os.access(p, os.W_OK)}
        raise KeyError(target_id)


# ----------------------------------------------------------------- hashing
def md5_listing_command(root: str, names: list[str] | None = None) -> str:
    """``md5sum``s + sizes of every file under ``root`` (relative paths, sorted),
    limited to entries starting with one of ``names``."""
    q = root if root.startswith("$HOME/") else shlex.quote(root)
    sel = ""
    if names:
        sel = " \\( " + " -o ".join(f"-path {shlex.quote('./' + n)}'*'" for n in names) + " \\)"
    return (f'cd "{q}" 2>/dev/null || cd {q} 2>/dev/null || exit 0; '
            f"find . -type f{sel} -printf '%s %p\\n' | sort -k2 | while read sz p; do "
            "echo \"$sz $(md5sum \"$p\" | cut -d' ' -f1) $p\"; done")  # fmt: skip


def parse_md5_listing(text: str) -> dict:
    """``{relpath: (size, md5)}``."""
    out = {}
    for line in (text or "").splitlines():
        parts = line.split(" ", 2)
        if len(parts) == 3 and parts[0].isdigit():
            out[parts[2].removeprefix("./")] = (int(parts[0]), parts[1])
    return out


def local_md5_listing(root: pathlib.Path, names: list[str] | None = None) -> dict:
    out = {}
    root = pathlib.Path(root)
    if not root.is_dir():
        return out
    for p in sorted(root.rglob("*")):
        if not p.is_file():
            continue
        rel = p.relative_to(root).as_posix()
        if names and not any(rel.startswith(n) for n in names):
            continue
        h = hashlib.md5()
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        out[rel] = (p.stat().st_size, h.hexdigest())
    return out


def compare(src: dict, dst: dict) -> dict:
    missing = sorted(set(src) - set(dst))
    differ = sorted(k for k in set(src) & set(dst) if src[k] != dst[k])
    return {"files": len(src), "bytes": sum(v[0] for v in src.values()), "missing": missing,
            "differ": differ, "ok": not missing and not differ}  # fmt: skip


# ----------------------------------------------------------------- the job
def offload_run(job, *, helpers, store, run: dict, target: dict, base: dict, delete_after: bool = False,
                clock=time.time) -> dict:  # fmt: skip
    """Copy one run to ``target`` and verify it.  ``base`` = the base station
    machine dict (local).  ``job`` may be None (tests)."""

    def say(level, text):
        if job is not None:
            job.log(level, text)

    def progress(key):
        return (lambda pct: job.set_progress(key, pct)) if job is not None else None

    exp_id, run_id = run["experiment"], run["id"]
    if target["kind"] == "disk":
        dst_user, dst_ip = base.get("user") or getpass.getuser(), "127.0.0.1"
        dst_root = f"{target['path'].rstrip('/')}/{exp_id}/{run_id}"
    else:
        dst_user, dst_ip = target["user"], target["host"]
        dst_root = f"{target['path'].rstrip('/')}/{exp_id}/{run_id}"
    manifest = {"t": clock(), "target": target["id"], "dest": dst_root, "machines": {}, "ok": True}

    # 1. the run folder (base station, local)
    run_dir = store.run_dir(exp_id, run_id)
    say("heading", f"Run folder -> {dst_root}/run")
    ok, method, msg = helpers.rsync_transfer(base.get("user"), "127.0.0.1", str(run_dir), dst_user, dst_ip,
                                             f"{dst_root}/run", progress_callback=progress("run"))  # fmt: skip
    if not ok:
        say("error", f"run folder: {msg}")
        manifest["ok"] = False

    # 2. each machine's bag segments
    bags_rel = f"adt4_output/experiments/{exp_id}/{run_id}/bags"
    for name, m in (run.get("machines") or {}).items():
        segs = m.get("segments") or []
        if not segs:
            continue
        mm = m["machine"]
        if job is not None:
            job.check_cancelled()
        say("heading", f"{name}: {', '.join(segs)} ({(m.get('size_bytes') or 0) / 1024**3:.2f} GB)")
        src_ip = "127.0.0.1" if mm.get("role") == "base_station" else mm.get("ip")
        relay_tmp = str(store.root / ".relay")
        ok, method, msg = helpers.rsync_transfer(mm.get("user"), src_ip, f"~/{bags_rel}", dst_user, dst_ip,
                                                 f"{dst_root}/bags", relay=False, relay_tmp=relay_tmp,
                                                 progress_callback=progress(name))  # fmt: skip
        entry = {"segments": segs, "copied": ok, "method": method, "error": None if ok else msg}
        if not ok:
            say("error", f"{name}: copy failed: {msg}")
            manifest["ok"] = False
            manifest["machines"][name] = entry
            continue
        # 3. verify: size + md5 on both sides, this machine's files only
        say("info", f"{name}: verifying (md5)")
        names = [f"{s}" for s in segs]
        rc, out, err = helpers.ssh_cmd(mm.get("user"), src_ip, md5_listing_command(f"$HOME/{bags_rel}", names), timeout=3600)
        src = parse_md5_listing(out) if rc == 0 else {}
        if target["kind"] == "disk":
            dst = local_md5_listing(pathlib.Path(f"{dst_root}/bags"), names)
        else:
            rc2, out2, _ = helpers.ssh_cmd(dst_user, dst_ip, md5_listing_command(f"{dst_root}/bags", names), timeout=3600)
            dst = parse_md5_listing(out2) if rc2 == 0 else {}
        cmp_ = compare(src, dst)
        if not src:
            cmp_["ok"] = False
            cmp_["error"] = f"could not list the source files ({err or rc})"
        entry["verify"] = cmp_
        if cmp_["ok"]:
            say("ok", f"{name}: {cmp_['files']} files, {cmp_['bytes'] / 1024**3:.2f} GB verified")
        else:
            say("error", f"{name}: verify FAILED: missing {cmp_['missing'][:5]} differ {cmp_['differ'][:5]}")
            manifest["ok"] = False
        manifest["machines"][name] = entry

    # 4. optional delete on the robots (only verified machines; never the base run folder)
    if delete_after:
        verified = [n for n, e in manifest["machines"].items() if (e.get("verify") or {}).get("ok")]
        if verified and job is not None:
            ans = job.confirm(
                f"Delete the verified bags of run {run_id} from {', '.join(verified)}?",
                kind="danger", required_text="delete", yes_label="Delete", no_label="Keep",
                detail={"machines": verified, "dest": dst_root},
            )  # fmt: skip
        else:
            ans = bool(verified) and job is None
        for n in verified if ans else []:
            m = run["machines"][n]
            mm = m["machine"]
            src_ip = "127.0.0.1" if mm.get("role") == "base_station" else mm.get("ip")
            paths = " ".join(f'"$HOME/{bags_rel}/{s}" "$HOME/{bags_rel}/{s}".*' for s in m["segments"])
            rc, _, err = helpers.ssh_cmd(mm.get("user"), src_ip, f"rm -rf {paths}", timeout=120)
            manifest["machines"][n]["deleted"] = rc == 0
            say("ok" if rc == 0 else "error", f"{n}: {'deleted' if rc == 0 else 'delete failed: ' + err}")

    # 5. manifest on the target + in the run
    tmp = store.run_dir(exp_id, run_id) / "offload_manifest.json"
    write_json(tmp, manifest)
    helpers.rsync_transfer(base.get("user"), "127.0.0.1", str(store.run_dir(exp_id, run_id)), dst_user, dst_ip,
                           f"{dst_root}/run")  # fmt: skip
    return manifest
