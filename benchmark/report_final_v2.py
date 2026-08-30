#!/usr/bin/env python3
"""Final report tables for debug.md task 14 (results_final_v2)."""

import csv
from collections import defaultdict
from statistics import mean

LIMIT = 10.0

rows = list(csv.DictReader(open("results_final_v2/rows.csv")))
for r in rows:
    # Budget accounting: carrier/carrier_b0/carrier_b1 enforce the limit
    # INTERNALLY (hard deadline; anytime deliberately uses the full budget),
    # and row wall time additionally contains subprocess + authoritative
    # re-validation overhead — no reclassification.  b4 has no internal
    # limiter and CREST's -t can overshoot: reclassify those by wall.
    if (r["status"] == "ok" and r["method"] in ("b4", "crest_base",
                                                "crest_full")
            and float(r["runtime_sec"]) > LIMIT + 0.6):
        r["status"] = "over_limit"
        r["success"] = "0"

def fam_group(f):
    if f.startswith("sweep_"):
        return "sweep"
    return {
        "instances_small_scramble": "small",
        "instances_small_ddmapd": "small",
        "instances_standard_scramble": "std",
        "instances_standard_ddmapd": "std",
        "instances_paper_r2r_m": "r2r_m",
        "instances_paper_dne_m": "dne_m",
        "instances_paper_s2w_m": "s2w_m",
    }.get(f, f)

methods = ["carrier", "carrier_b0", "carrier_b1", "b4", "crest_base",
           "crest_full", "natcbs"]
groups = ["small", "std", "sweep", "r2r_m", "dne_m", "s2w_m"]

agg = defaultdict(lambda: {"ok": 0, "n": 0, "mk": [], "soc": [], "fs": []})
for r in rows:
    a = agg[(fam_group(r["family"]), r["method"])]
    a["n"] += 1
    if r["success"] == "1":
        a["ok"] += 1
        a["mk"].append(float(r["executed_makespan"]))
        if r["weighted_soc"]:
            a["soc"].append(float(r["weighted_soc"]))
        if r.get("first_solution_ms") not in ("", "-1", None):
            a["fs"].append(float(r["first_solution_ms"]))

print("== success (solved/n | mean makespan) ==")
hdr = f"{'group':8s}" + "".join(f"{m:>20s}" for m in methods)
print(hdr)
for g in groups:
    line = f"{g:8s}"
    for m in methods:
        a = agg.get((g, m))
        if not a or a["n"] == 0:
            line += f"{'-':>20s}"
            continue
        mk = f"{mean(a['mk']):.0f}" if a["mk"] else "-"
        line += f"{a['ok']:>4d}/{a['n']:<4d}{mk:>10s} "
    print(line)

tot = defaultdict(lambda: [0, 0])
for (g, m), a in agg.items():
    tot[m][0] += a["ok"]
    tot[m][1] += a["n"]
print("\n== totals (unified 10 s budget) ==")
for m in methods:
    s, n = tot[m]
    print(f"  {m:12s} {s}/{n}")

# carrier first-solution latency by group
print("\n== carrier mean first-solution ms by group ==")
for g in groups:
    a = agg.get((g, "carrier"))
    if a and a["fs"]:
        print(f"  {g:8s} {mean(a['fs']):9.1f} ms")

# ablation summary
print("\n== ablation (solved count over 9 dev cases; mean mk on common) ==")
ab = list(csv.DictReader(open("results_ablation/ablation_rows.csv")))
by_var = defaultdict(lambda: {"ok": 0, "mk": {}})
for r in ab:
    v = by_var[r["variant"]]
    if r["solved"] == "1":
        v["ok"] += 1
        v["mk"][r["case"]] = float(r["makespan"])
common = None
for v in by_var.values():
    ks = set(v["mk"])
    common = ks if common is None else (common & ks)
for name in ["full", "no_macro", "strict_inv", "no_yield", "no_astar",
             "b0", "b1"]:
    v = by_var[name]
    cm = mean([v["mk"][k] for k in common]) if common else 0
    print(f"  {name:11s} solved {v['ok']}/9   mean_mk(common {len(common)})="
          f"{cm:.0f}")
