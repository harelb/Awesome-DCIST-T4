#!/usr/bin/env python3
"""Renumber a loop-closure json (robot ids = row order of a sessions file, from 0) to another
sessions file; closures touching a session that is not in the new list are dropped.
    remap_lc.py OLD_SESSIONS NEW_SESSIONS IN.json OUT.json"""
import json, sys
names = lambda f: [l.split()[0] for l in open(f) if l.strip() and not l.startswith("#")]
old, new = names(sys.argv[1]), names(sys.argv[2])
idx = {n: i for i, n in enumerate(new)}
m = {i: idx[n] for i, n in enumerate(old) if n in idx}
out = []
for lc in json.load(open(sys.argv[3])):
    if lc["robot_from"] in m and lc["robot_to"] in m:
        out.append(dict(lc, robot_from=m[lc["robot_from"]], robot_to=m[lc["robot_to"]]))
json.dump(out, open(sys.argv[4], "w"))
print(f"{len(out)} closures kept; id map {m}")
