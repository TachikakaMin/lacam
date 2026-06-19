#!/usr/bin/env python3
"""Plot throughput curves for distribution/K/agent-count sweep."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


DISPLAY_NAME = {
    "50_50": "50/50",
    "80_20_inbound": "80/20",
}


def safe_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("./build/results/goal3_distribution_k_sweep/throughput_sweep_results.csv"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("./build/results/goal3_distribution_k_sweep/throughput_curves.png"),
    )
    args = parser.parse_args()

    with args.input.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"no rows in {args.input}")

    grouped: dict[tuple[str, int], list[tuple[int, float]]] = defaultdict(list)
    distributions: list[str] = []
    capacities: list[int] = []
    for row in rows:
        distribution = row["distribution"]
        capacity = int(row.get("requested_capacity") or row["multi_carry_capacity"])
        agents = int(row["num_agents"])
        throughput = safe_float(row["throughput"])
        grouped[(distribution, capacity)].append((agents, throughput))
        if distribution not in distributions:
            distributions.append(distribution)
        if capacity not in capacities:
            capacities.append(capacity)

    distributions.sort(key=lambda name: {"50_50": 0, "80_20_inbound": 1}.get(name, 99))
    capacities.sort()
    fig, axes = plt.subplots(1, len(distributions), figsize=(12, 4.8), sharey=True)
    if len(distributions) == 1:
        axes = [axes]

    markers = {1: "o", 2: "s", 3: "^"}
    colors = {1: "#2f6fbb", 2: "#d04f3a", 3: "#2f8f5b"}
    for ax, distribution in zip(axes, distributions):
        for capacity in capacities:
            points = sorted(grouped.get((distribution, capacity), []))
            if not points:
                continue
            agents = [p[0] for p in points]
            throughput = [p[1] for p in points]
            ax.plot(
                agents,
                throughput,
                marker=markers.get(capacity, "o"),
                color=colors.get(capacity),
                linewidth=2.2,
                markersize=6,
                label=f"K={capacity}",
            )
        ax.set_title(f"Distribution {DISPLAY_NAME.get(distribution, distribution)}")
        ax.set_xlabel("Agents")
        ax.set_xticks(sorted({int(row["num_agents"]) for row in rows}))
        ax.grid(True, color="#d9dde3", linewidth=0.8, alpha=0.8)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    axes[0].set_ylabel("Throughput (completed tasks / timestep)")
    axes[-1].legend(frameon=False, title="Capacity")
    fig.suptitle("Symbotic Star Throughput by Distribution, K, and Agent Count")
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
