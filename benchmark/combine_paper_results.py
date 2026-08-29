#!/usr/bin/env python3
"""Combine paper-suite result CSVs and print per-(family, method) stats."""

import csv
from collections import defaultdict
from statistics import mean

SRCS = {
    "results_paper_main/rows.csv": None,
    "results_paper_full3600/rows.csv": "crest_full_3600",
    "results_paper_s2w_b4/rows.csv": None,
    "results_s2w_probe/rows.csv": "crest_base_w5",
}


def main():
    rows = []
    for path, rename in SRCS.items():
        for r in csv.DictReader(open(path)):
            if rename:
                r["method"] = rename
            rows.append(r)
    with open("results_paper_combined.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    agg = defaultdict(lambda: {"ok": 0, "n": 0, "mk": [], "rt": []})
    for r in rows:
        a = agg[(r["family"], r["method"])]
        a["n"] += 1
        if r["status"] == "ok":
            a["ok"] += 1
            a["mk"].append(int(r["executed_makespan"]))
            a["rt"].append(float(r["runtime_sec"]))

    print(f"{'family':8s} {'method':16s} solved   mean_mk  mean_rt_s")
    for (fam, m), a in sorted(agg.items()):
        mk = f"{mean(a['mk']):9.1f}" if a["mk"] else "        -"
        rt = f"{mean(a['rt']):9.1f}" if a["rt"] else "        -"
        print(f"{fam:8s} {m:16s} {a['ok']:>2d}/{a['n']:<3d} {mk} {rt}")
    print(f"\ntotal rows: {len(rows)} -> results_paper_combined.csv")


if __name__ == "__main__":
    main()
