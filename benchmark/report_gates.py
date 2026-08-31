#!/usr/bin/env python3
"""Gate A/B/C reports for the goal-set tau layer (debug.md v4 section 6).

Gate A: singleton parity — per-instance carrier rows of results_gateA vs
        the official results_integrated_v2 (success set +-2 seed-swap
        tolerance; per-instance makespan equality on common solved).
Gate B: BRaP-B dynamic tau vs the three static controls.
"""
import csv
import re
import sys
from pathlib import Path

BENCH = Path(__file__).resolve().parent


def load(path, method="carrier", by_name=False):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["method"] != method:
                continue
            key = r["instance"] if by_name else (r["family"], r["instance"])
            rows[key] = r
    return rows


def gate_a():
    old = load(BENCH / "results_integrated_v2/rows.csv")
    new = load(BENCH / "results_gateA/rows.csv")
    print(f"== Gate A: {len(new)} instances (v2 baseline {len(old)}) ==")
    s_old = {k for k, r in old.items() if r["success"] == "1"}
    s_new = {k for k, r in new.items() if r["success"] == "1"}
    print(f"solved: v2 {len(s_old)}  now {len(s_new)}")
    lost = sorted(s_old - s_new)
    won = sorted(s_new - s_old)
    print(f"lost vs v2 ({len(lost)}): {lost}")
    print(f"won vs v2 ({len(won)}): {won}")
    mk_diff = []
    for k in sorted(s_old & s_new):
        a, b = old[k], new[k]
        if a["executed_makespan"] != b["executed_makespan"]:
            mk_diff.append((k, a["executed_makespan"], b["executed_makespan"]))
    print(f"makespan diffs on common solved ({len(mk_diff)}):")
    for k, a, b in mk_diff:
        print(f"  {k}: v2={a} now={b}")
    ok = len(s_new) >= len(s_old) - 2 and len(lost) <= 2
    print(f"GATE A (success-set +-2): {'PASS' if ok else 'FAIL'}; "
          f"exact-makespan rows: {len(s_old & s_new) - len(mk_diff)}"
          f"/{len(s_old & s_new)}")


def gate_b():
    print("\n== Gate B: BRaP-B dynamic tau (pool) vs static controls ==")
    pool = load(BENCH / "results_gateB/rows.csv", by_name=True)
    static = load(BENCH / "results_brap/rows.csv", by_name=True)
    if not pool:
        print("(pool results missing)")
        return
    # match pool B-type rows to their static counterparts by stripping
    # the _pool suffix; R1 rows are the within-suite singleton control
    b_pool = {k: r for k, r in pool.items() if k.endswith("_pool")}
    r1_pool = {k: r for k, r in pool.items() if not k.endswith("_pool")}
    sp = sum(r["success"] == "1" for r in b_pool.values())
    print(f"B-type pool: {sp}/{len(b_pool)} solved")
    # mechanical size gate (design_final 8.2 / debug.md v4 section 6):
    # the success gate is defined on <=10x10 ONLY; >=20x20 is the
    # declared horizon-wall regime (record-only, no gate)
    def size_of(name):
        m = re.match(r"brap_h(\d+)w(\d+)_", name)
        return (int(m[1]), int(m[2]))
    small = {k: r for k, r in pool.items()
             if max(size_of(k)) <= 10}
    large = {k: r for k, r in pool.items()
             if max(size_of(k)) > 10}
    s_small = sum(r["success"] == "1" for r in small.values())
    s_large = sum(r["success"] == "1" for r in large.values())
    sb = sum(r["success"] == "1" for k, r in small.items()
             if k.endswith("_pool"))
    nb = sum(1 for k in small if k.endswith("_pool"))
    print(f"<=10x10 (GATED): {s_small}/{len(small)} solved "
          f"(B-type {sb}/{nb}); gate: >= 34 -> "
          f"{'PASS' if s_small >= 34 else 'FAIL'}")
    print(f">=20x20 (record-only, horizon wall): "
          f"{s_large}/{len(large)} solved")
    ss = 0
    mk_rows = []
    for k, r in sorted(b_pool.items()):
        k0 = k[: -len("_pool")]
        r0 = static.get(k0)
        if r0 is None:
            continue
        if r0["success"] == "1":
            ss += 1
        if r["success"] == "1" and r0["success"] == "1":
            mk_rows.append((k0, int(r0["executed_makespan"]),
                            int(r["executed_makespan"])))
    print(f"B-type static counterpart solved: {ss}")
    if mk_rows:
        import statistics
        ratios = [b / a for _, a, b in mk_rows]
        print(f"common solved ({len(mk_rows)}) mk pool/static: "
              f"mean {statistics.mean(ratios):.3f}  "
              f"median {statistics.median(ratios):.3f}")
        for k, a, b in mk_rows:
            print(f"  {k}: static={a} pool={b}  ({b/a:.3f}x)")
    # R1 singleton control: per-instance equality vs results_brap
    diffs = 0
    common = 0
    for k, r in r1_pool.items():
        r0 = static.get(k)
        if r0 is None:
            continue
        common += 1
        if (r["success"], r["executed_makespan"]) != (
                r0["success"], r0["executed_makespan"]):
            diffs += 1
            print(f"  R1 drift {k}: static "
                  f"({r0['success']},{r0['executed_makespan']}) pool-run "
                  f"({r['success']},{r['executed_makespan']})")
    print(f"R1 singleton control: {common} common rows, {diffs} drift")


if __name__ == "__main__":
    if "--gate-b-only" not in sys.argv:
        gate_a()
    if (BENCH / "results_gateB/rows.csv").exists():
        gate_b()
