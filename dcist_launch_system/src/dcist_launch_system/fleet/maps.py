"""Experiment / prior-map helpers shared by the TUI, CLI and web console."""

from __future__ import annotations

from dataclasses import dataclass

# A run directory is usable as a prior map when both of these were saved.
PRIOR_SUBDIRS = ("hydra", "roman")


def is_prior_map(experiment: dict) -> bool:
    """``experiment`` is an entry from ``list_remote_experiments``."""
    subdirs = experiment.get("subdirs", [])
    return all(d in subdirs for d in PRIOR_SUBDIRS)


@dataclass
class FileComparison:
    path: str
    hashes: dict[str, str | None]  # machine -> md5 (None = missing)
    consistent: bool

    def cell(self, machine: str) -> str:
        """Display value: ``MISSING``, ``OK`` or the first 8 hex digits."""
        h = self.hashes[machine]
        if h is None:
            return "MISSING"
        return "OK" if self.consistent else h[:8]


def compare_hashes(machine_hashes: dict[str, dict[str, str]]) -> list[FileComparison]:
    """Compare per-machine ``{relative_path: md5}`` maps file by file.

    A file is consistent only if every machine has it with the same hash.
    """
    machines = sorted(machine_hashes)
    files = sorted({f for h in machine_hashes.values() for f in h})
    rows = []
    for path in files:
        hashes = {mn: machine_hashes[mn].get(path) for mn in machines}
        present = [h for h in hashes.values() if h]
        consistent = len(present) == len(machines) and len(set(present)) == 1
        rows.append(FileComparison(path, hashes, consistent))
    return rows
