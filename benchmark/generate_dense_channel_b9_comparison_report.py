#!/usr/bin/env python3
"""Render the dense-b9 carrier vs carrier_brd page with the original UI."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from generate_current_carrier_vs_brd_like_original import (
    ARCHIVE_GENERATOR,
    common_first_solution_geometric_ratio,
    load_original,
)


HERE = Path(__file__).resolve().parent
INSTANCE_FAMILY = "dense_channel_b9_agent_task_v1"
INSTANCE_ROOT = (
    HERE
    / "viz_web"
    / "dense_channel_b9_agent_task_suite_v1_20260914"
    / "instances"
)
DEFAULT_CARRIER_RUN = HERE / "results_dense_channel_b9_agent_task_v1_20260914"
DEFAULT_BRD_RUN = HERE / "results_dense_channel_b9_agent_task_brd_v1_20260914"
DEFAULT_OUTPUT = HERE / "viz_web/dense_channel_b9_carrier_vs_brd_v1_20260914"
FROZEN_PROTOCOL = {
    "n_tasks": 24,
    "jobs": 14,
    "timeout_per_run_sec": 10,
    "solver_seed": 0,
    "objective_weights": {
        "alpha": 1.0,
        "beta": 1.0,
        "gamma": 1.0,
        "delta": 1.0,
    },
    "following": "allowed",
}
FROZEN_BINARY_SHA256 = (
    "d7d76608ba49716f6d07e1b841b8aa729ea73666e6611c68885ee5b949dbe49f"
)


def validate_matching_protocol(carrier_timing: dict, brd_timing: dict) -> None:
    for key, expected in FROZEN_PROTOCOL.items():
        for label, timing in (
            ("carrier", carrier_timing),
            ("carrier_brd", brd_timing),
        ):
            if timing.get(key) != expected:
                raise ValueError(
                    "{} {} violates frozen protocol: {!r} != {!r}".format(
                        label, key, timing.get(key), expected
                    )
                )
        if carrier_timing.get(key) != brd_timing.get(key):
            raise ValueError(
                "cross-run protocol mismatch for {}: {!r} != {!r}".format(
                    key, carrier_timing.get(key), brd_timing.get(key)
                )
            )


def validate_binary_provenance(carrier_timing: dict, brd_timing: dict) -> None:
    commits = []
    for label, timing in (
        ("carrier", carrier_timing),
        ("carrier_brd", brd_timing),
    ):
        provenance = timing.get("provenance", {})
        digest = provenance.get("binary_sha256")
        if digest != FROZEN_BINARY_SHA256:
            raise ValueError(
                "{} binary_sha256 is not the reviewed binary".format(label)
            )
        commit = provenance.get("git_commit")
        if not commit:
            raise ValueError("{} git_commit is missing".format(label))
        commits.append(commit)
    if commits[0] != commits[1]:
        raise ValueError("cross-run git_commit mismatch")


def validate_exact_rows(
    carrier_rows_path: Path,
    brd_rows_path: Path,
    manifest_path: Path,
    expected_count: int = None,
) -> None:
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    expected = [row["id"] for row in manifest]
    if len(expected) != len(set(expected)):
        raise ValueError("manifest must contain unique instances")
    if expected_count is not None and len(expected) != expected_count:
        raise ValueError(
            "manifest must contain {} instances".format(expected_count)
        )

    def validate(path: Path, method: str) -> None:
        with Path(path).open(encoding="utf-8", newline="") as source:
            rows = list(csv.DictReader(source))
        names = [row.get("instance") for row in rows]
        if len(names) != len(set(names)):
            raise ValueError("duplicate instance in {} rows".format(method))
        if len(names) != len(expected) or set(names) != set(expected):
            raise ValueError("{} rows differ from manifest".format(method))
        for row in rows:
            if row.get("method") != method:
                raise ValueError("wrong method in {} rows".format(method))
            if row.get("family") != INSTANCE_FAMILY:
                raise ValueError("wrong family in {} rows".format(method))

    validate(carrier_rows_path, "carrier")
    validate(brd_rows_path, "carrier_brd")

def _replace_required(text: str, source: str, target: str) -> str:
    if source not in text:
        raise ValueError("missing archived page marker: {}".format(source))
    return text.replace(source, target)


def specialize_page(text: str, english: bool = False) -> str:
    if english:
        replacements = (
            (
                "Carrier vs baseline (carrier_brd) — full-509 "
                "first-solution time and solution quality",
                "Carrier vs baseline (carrier_brd) — dense-b9 Agent×Task "
                "24 cases: our first solution vs baseline final",
            ),
            (
                '<a href="index.html">中文版</a>',
                '<a href="index.html">中文版</a> · '
                '<a href="../dense_channel_b9_agent_task_benchmark_v1_20260914/'
                'index.html">Carrier-only matrix</a> · '
                '<a href="../dense_channel_b9_agent_task_suite_v1_20260914/'
                'index.html">Scenario list</a>',
            ),
            (
                "First-solution time (ms, log-log)",
                "Solve time (our first solution / baseline final, ms, log-log)",
            ),
            (
                "Baseline first-solution time (ms)",
                "Baseline final time (ms)",
            ),
        )
        optional = (
            ("`first ${s.first} ms, mk ${s.mk}, soc ${s.soc}`",
             "`time ${s.first} ms, mk ${s.mk}, soc ${s.soc}`"),
            ("`carrier：${f(r.c)}<br>baseline：${f(r.b)}<br>`",
             "`Carrier (first global goal): ${f(r.c)}<br>"
             "Baseline (final): ${f(r.b)}<br>`"),
            ("function sideInfoText(s) {", "function sideInfoText(s, side) {"),
            ("? `Solved: first solution ${s.first} ms ｜ makespan ${s.mk} ｜ SOC ${s.soc}`",
             "? `${side === \"carrier\" ? \"First global goal\" : \"Final\"}: "
             "time ${s.first} ms ｜ makespan ${s.mk} ｜ SOC ${s.soc}`"),
            ("sideInfoText(r.c));", 'sideInfoText(r.c, "carrier"));'),
            ("sideInfoText(r.b));", 'sideInfoText(r.b, "baseline"));'),
        )
    else:
        replacements = (
            (
                "carrier vs baseline(carrier_brd) — full 509 首解时间与解质量",
                "carrier vs baseline(carrier_brd) — dense b9 Agent×Task "
                "24例：我们首解 vs baseline final",
            ),
            (
                '<a href="index_en.html">English version</a>',
                '<a href="index_en.html">English version</a> · '
                '<a href="../dense_channel_b9_agent_task_benchmark_v1_20260914/'
                'index.html">carrier 单方法矩阵</a> · '
                '<a href="../dense_channel_b9_agent_task_suite_v1_20260914/'
                'index.html">场景清单</a>',
            ),
            (
                "首解时间（ms，log-log）",
                "求解时间（我们首解 / baseline final，ms，log-log）",
            ),
            (
                "baseline 首解时间 (ms)",
                "baseline final 时间 (ms)",
            ),
        )
        optional = (
            ("`首解 ${s.first} ms，mk ${s.mk}，soc ${s.soc}`",
             "`时间 ${s.first} ms，mk ${s.mk}，soc ${s.soc}`"),
            ("`carrier：${f(r.c)}<br>baseline：${f(r.b)}<br>`",
             "`carrier（首个全局 goal）：${f(r.c)}<br>"
             "baseline（final）：${f(r.b)}<br>`"),
            ("function sideInfoText(s) {", "function sideInfoText(s, side) {"),
            ("? `解出：首解 ${s.first} ms ｜ makespan ${s.mk} ｜ soc ${s.soc}`",
             "? `${side === \"carrier\" ? \"首个全局 goal\" : \"final\"}："
             "时间 ${s.first} ms ｜ makespan ${s.mk} ｜ soc ${s.soc}`"),
            ("sideInfoText(r.c));", 'sideInfoText(r.c, "carrier"));'),
            ("sideInfoText(r.b));", 'sideInfoText(r.b, "baseline"));'),
        )
    for source, target in replacements:
        text = _replace_required(text, source, target)
    for source, target in optional:
        text = text.replace(source, target)
    return text


def inject_protocol_banner(
    text: str, carrier_timing: dict, brd_timing: dict, english: bool = False
) -> str:
    validate_matching_protocol(carrier_timing, brd_timing)
    validate_binary_provenance(carrier_timing, brd_timing)
    provenance = carrier_timing["provenance"]
    commit = provenance["git_commit"]
    digest = provenance["binary_sha256"]
    if english:
        content = (
            "Fair protocol: same 24 test cases; same commit <code>{}</code>; "
            "same reviewed binary SHA-256 <code>{}</code>; 10 seconds per case; "
            "14 workers; solver seed 0; weights (1,1,1,1); following allowed."
        ).format(commit, digest)
    else:
        content = (
            "公平协议：双方使用同一 24 个 testcase、同一 commit "
            "<code>{}</code>、同一 reviewed binary SHA-256 <code>{}</code>；"
            "每例 10 秒；14 workers；solver seed 0；weights (1,1,1,1)；"
            "following allowed。"
        ).format(commit, digest)
    block = (
        '<div id="protocol" style="font-size:12px;color:#9aa3b2;'
        'margin:0 0 10px;overflow-wrap:anywhere">{}</div>'.format(content)
    )
    marker = '<div id="prov"></div>'
    return _replace_required(text, marker, marker + "\n" + block)


def validate_baseline_final_times(rows_path: Path) -> None:
    with Path(rows_path).open(encoding="utf-8", newline="") as source:
        for row in csv.DictReader(source):
            if row.get("method") != "carrier_brd":
                raise ValueError("baseline rows contain the wrong method")
            if row.get("success") != "1":
                continue
            first = row.get("first_solution_ms")
            final = row.get("deliverable_ms")
            if first in (None, "") or final in (None, ""):
                raise ValueError("baseline final times must be non-empty")
            if first != final:
                raise ValueError(
                    "baseline must expose final time for {}".format(
                        row["instance"]
                    )
                )


def update_dense_comparison_semantics(out_dir: Path, time_ratio: float) -> None:
    replacements = {
        "index.html": (
            "对角线下方 = carrier 数值更小。",
            "对角线上方 = carrier 数值更小。<br>"
            "<strong>时间几何比（carrier 首个全局 goal / baseline final）"
            "= {:.3f}，小于 1 表示 carrier 更快拿到首个全局解。</strong>".format(
                time_ratio
            ),
        ),
        "index_en.html": (
            "Below the diagonal = a lower Carrier value.",
            "Above the diagonal = a lower Carrier value.<br>"
            "<strong>Time geometric ratio (Carrier first global goal / "
            "baseline final) = {:.3f}; values below 1 mean Carrier reaches "
            "its first global solution sooner.</strong>".format(time_ratio),
        ),
    }
    for name, (source, target) in replacements.items():
        page = Path(out_dir) / name
        content = page.read_text(encoding="utf-8")
        page.write_text(
            _replace_required(content, source, target), encoding="utf-8"
        )

def main() -> None:
    if not ARCHIVE_GENERATOR.is_file():
        raise FileNotFoundError(
            "original comparison generator not found: {}".format(
                ARCHIVE_GENERATOR
            )
        )
    parser = argparse.ArgumentParser()
    parser.add_argument("--carrier-run", type=Path, default=DEFAULT_CARRIER_RUN)
    parser.add_argument("--brd-run", type=Path, default=DEFAULT_BRD_RUN)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    carrier_timing = json.loads(
        (args.carrier_run / "timing.json").read_text(encoding="utf-8")
    )
    brd_timing = json.loads(
        (args.brd_run / "timing.json").read_text(encoding="utf-8")
    )
    validate_matching_protocol(carrier_timing, brd_timing)
    validate_binary_provenance(carrier_timing, brd_timing)
    validate_exact_rows(
        args.carrier_run / "rows.csv",
        args.brd_run / "rows.csv",
        INSTANCE_ROOT.parent / "manifest.json",
        expected_count=24,
    )
    validate_baseline_final_times(args.brd_run / "rows.csv")

    original = load_original()
    original.REPO = HERE
    original.INSTANCE_ROOTS = {INSTANCE_FAMILY: INSTANCE_ROOT}
    original.CARRIER_RUN = args.carrier_run.resolve()
    original.BRD_RUN = args.brd_run.resolve()

    out_dir = args.out_dir.resolve()
    sys.argv = [
        str(ARCHIVE_GENERATOR),
        "--carrier-run", str(original.CARRIER_RUN),
        "--brd-run", str(original.BRD_RUN),
        "--out-dir", str(out_dir),
    ]
    original.main()

    for name, english in (("index.html", False), ("index_en.html", True)):
        page = out_dir / name
        specialized = specialize_page(
            page.read_text(encoding="utf-8"), english
        )
        specialized = inject_protocol_banner(
            specialized, carrier_timing, brd_timing, english
        )
        page.write_text(specialized, encoding="utf-8")
    update_dense_comparison_semantics(
        out_dir,
        common_first_solution_geometric_ratio(args.carrier_run, args.brd_run),
    )
    print("dense comparison page={}".format(out_dir / "index.html"))


if __name__ == "__main__":
    main()
