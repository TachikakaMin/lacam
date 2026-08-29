#!/usr/bin/env python3
"""Final baseline-vs-carrier comparison at the unified 10 s budget.
Any successful row with runtime_sec > limit is re-classified as over_limit
(b4 has no hard in-process cutoff; CREST's internal cutoff can overshoot)."""

import csv
from collections import defaultdict
from statistics import mean

LIMIT = 10.0

rows = list(csv.DictReader(open("results_final10s/rows.csv")))
for r in rows:
    if r["status"] == "ok" and float(r["runtime_sec"]) > LIMIT:
        r["status"] = "over_limit"
        r["success"] = "0"

fam_of = {
    "instances_small_scramble": "small_scr",
    "instances_small_ddmapd": "small_dd",
    "instances_standard_scramble": "std_scr",
    "instances_standard_ddmapd": "std_dd",
    "instances_paper_r2r_m": "r2r_m",
    "instances_paper_dne_m": "dne_m",
    "instances_paper_s2w_m": "s2w_m",
}
agg = defaultdict(lambda: {"ok": 0, "n": 0, "mk": [], "rt": []})
for r in rows:
    fam = fam_of.get(r["family"], r["family"])
    a = agg[(fam, r["method"])]
    a["n"] += 1
    if r["success"] == "1":
        a["ok"] += 1
        a["mk"].append(int(r["executed_makespan"]))
        a["rt"].append(float(r["runtime_sec"]))

methods = ["carrier", "b4", "crest_base", "crest_full", "natcbs"]
fams = ["small_scr", "small_dd", "std_scr", "std_dd", "r2r_m", "dne_m",
        "s2w_m"]
print(f"{'family':10s}" + "".join(f"{m:>22s}" for m in methods))
print(" " * 10 + "".join(f"{'solved  mk~  rt~':>22s}" for _ in methods))
for f in fams:
    line = f"{f:10s}"
    for m in methods:
        a = agg.get((f, m))
        if a is None or a["n"] == 0:
            line += f"{'-':>22s}"
            continue
        mk = f"{mean(a['mk']):.0f}" if a["mk"] else "-"
        rt = f"{mean(a['rt']):.1f}" if a["rt"] else "-"
        line += f"{a['ok']:>3d}/{a['n']:<3d}{mk:>8s}{rt:>8s}"
    print(line)

tot = defaultdict(lambda: [0, 0])
for (f, m), a in agg.items():
    tot[m][0] += a["ok"]
    tot[m][1] += a["n"]
print("\ntotals (10 s budget):")
for m in methods:
    s, n = tot[m]
    print(f"  {m:12s} {s}/{n}")
