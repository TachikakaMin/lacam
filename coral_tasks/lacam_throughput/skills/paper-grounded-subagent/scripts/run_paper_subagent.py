#!/usr/bin/env python3
"""启动论文支撑的 coding subagent 来处理 LaCAM throughput 工作。"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import subprocess
import sys
from pathlib import Path


PRIMARY_SUBAGENT_MODEL = "gpt-5.3-codex-spark"
PRIMARY_SUBAGENT_EFFORT = "high"
FALLBACK_SUBAGENT_MODEL = "gpt-5.6-luna"
FALLBACK_SUBAGENT_EFFORT = "xhigh"
CLAUDE_SUBAGENT_MODEL = "claude-opus-4-8"
CLAUDE_SUBAGENT_EFFORT = "medium"
PAPER_BRIEFS_ROOT = Path("coral_tasks/lacam_throughput/papers/briefs")
PAPER_README = Path("coral_tasks/lacam_throughput/papers/README.md")
PAPER_CATEGORIES = Path("coral_tasks/lacam_throughput/papers/idea_categories.md")
QUOTA_ERROR_PATTERNS = (
    "quota",
    "rate limit",
    "rate_limit",
    "insufficient credits",
    "insufficient_credit",
    "billing",
    "usage limit",
    "capacity",
    "no quota",
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="启动一个基于本地论文库来源的 coding subagent。"
    )
    parser.add_argument("--paper", required=True, help="支撑该 idea 的本地论文 brief 路径。")
    parser.add_argument("--idea", required=True, help="一句简洁的论文支撑 hypothesis。")
    parser.add_argument("--task", required=True, help="委托给 subagent 的具体工作。")
    parser.add_argument(
        "--out-dir",
        default="hl_agent/runs/paper_grounded_subagents",
        help="prompt、日志和报告输出目录。",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只写入 prompt/report 路径并打印命令，不实际启动。",
    )
    parser.add_argument(
        "--runtime",
        choices=("codex", "claude", "claude_code"),
        default=os.environ.get("PAPER_SUBAGENT_RUNTIME", "codex"),
        help="coding subagent runtime：codex 或 claude。默认可由 PAPER_SUBAGENT_RUNTIME 设置。",
    )
    parser.add_argument(
        "--codex-model",
        default=os.environ.get("PAPER_SUBAGENT_CODEX_MODEL", PRIMARY_SUBAGENT_MODEL),
        help="Codex subagent model；默认只是建议，可按任务选择其他可用模型。",
    )
    parser.add_argument(
        "--codex-effort",
        default=os.environ.get("PAPER_SUBAGENT_CODEX_EFFORT", PRIMARY_SUBAGENT_EFFORT),
        help="Codex subagent reasoning effort；默认 high。",
    )
    parser.add_argument(
        "--codex-fallback-model",
        default=os.environ.get(
            "PAPER_SUBAGENT_CODEX_FALLBACK_MODEL", FALLBACK_SUBAGENT_MODEL
        ),
        help="Codex quota/rate-limit fallback model；默认只是建议。",
    )
    parser.add_argument(
        "--codex-fallback-effort",
        default=os.environ.get(
            "PAPER_SUBAGENT_CODEX_FALLBACK_EFFORT", FALLBACK_SUBAGENT_EFFORT
        ),
        help="Codex fallback reasoning effort；默认 medium。",
    )
    parser.add_argument(
        "--claude-model",
        default=os.environ.get("PAPER_SUBAGENT_CLAUDE_MODEL", CLAUDE_SUBAGENT_MODEL),
        help="Claude coding subagent model；默认只是建议，可按任务选择其他可用模型。",
    )
    parser.add_argument(
        "--claude-effort",
        default=os.environ.get("PAPER_SUBAGENT_CLAUDE_EFFORT", CLAUDE_SUBAGENT_EFFORT),
        help="Claude coding policy label，默认 medium；Claude CLI 当前只接收 --model，label 写入日志。",
    )
    args = parser.parse_args()
    runtime = _normalize_runtime(args.runtime)

    repo = Path.cwd()
    paper_path = (repo / args.paper).resolve()
    if not paper_path.exists():
        print(f"找不到论文来源：{args.paper}", file=sys.stderr)
        return 2
    if not _is_paper_brief(repo, paper_path):
        print(
            "论文来源必须是本地 paper brief，位置应在 "
            f"{PAPER_BRIEFS_ROOT.as_posix()} 下；请先从 "
            f"{PAPER_README.as_posix()} 开始筛选，然后传入选定 brief",
            file=sys.stderr,
        )
        return 2

    out_root = repo / args.out_dir
    out_root.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    slug = _slug(args.idea)
    run_dir = out_root / f"{stamp}_{slug}"
    run_dir.mkdir(parents=True, exist_ok=False)

    report_path = run_dir / "report.md"
    prompt_path = run_dir / "prompt.md"
    primary_stdout_path = run_dir / f"{runtime}.primary.stdout.jsonl"
    primary_stderr_path = run_dir / f"{runtime}.primary.stderr.txt"
    fallback_stdout_path = run_dir / "codex.fallback.stdout.jsonl"
    fallback_stderr_path = run_dir / "codex.fallback.stderr.txt"

    prompt = _build_prompt(
        repo=repo,
        paper_arg=args.paper,
        paper_path=paper_path,
        idea=args.idea,
        task=args.task,
        report_path=report_path,
    )
    prompt_path.write_text(prompt)

    if runtime == "claude":
        primary_cmd = _claude_cmd(prompt, args.claude_model, args.claude_effort)
        fallback_cmd = []
        primary_label = f"{args.claude_model} / {args.claude_effort}"
        fallback_label = "(none)"
    else:
        primary_cmd = _codex_cmd(prompt, args.codex_model, args.codex_effort)
        fallback_cmd = _codex_cmd(
            prompt, args.codex_fallback_model, args.codex_fallback_effort
        )
        primary_label = f"{args.codex_model} / {args.codex_effort}"
        fallback_label = (
            f"{args.codex_fallback_model} / {args.codex_fallback_effort}"
        )

    print(f"论文来源：{args.paper}")
    print(f"prompt：{prompt_path}")
    print(f"报告：{report_path}")
    print(f"runtime：{runtime}")
    print(f"主模型：{primary_label}")
    print(f"fallback：{fallback_label}")
    print(f"主模型 stdout 日志：{primary_stdout_path}")
    print(f"主模型 stderr 日志：{primary_stderr_path}")
    if runtime == "codex":
        print(f"fallback stdout 日志：{fallback_stdout_path}")
        print(f"fallback stderr 日志：{fallback_stderr_path}")
    print("主模型命令：" + " ".join(_shell_quote(part) for part in primary_cmd))
    if fallback_cmd:
        print("fallback 命令：" + " ".join(_shell_quote(part) for part in fallback_cmd))
    if args.dry_run:
        return 0

    result = _run_subagent(primary_cmd, repo, primary_stdout_path, primary_stderr_path)
    used_fallback = False

    if (
        runtime == "codex"
        and result.returncode != 0
        and _looks_like_quota_error(primary_stdout_path, primary_stderr_path)
    ):
        print(
            "主模型 subagent 遇到 quota/rate-limit 类错误；"
            f"改用 {args.codex_fallback_model} "
            f"（{args.codex_fallback_effort} effort）重试"
        )
        used_fallback = True
        result = _run_subagent(fallback_cmd, repo, fallback_stdout_path, fallback_stderr_path)

    print(f"subagent 退出码：{result.returncode}")
    if used_fallback:
        print(
            "已使用 fallback 模型："
            f"{args.codex_fallback_model} / {args.codex_fallback_effort}"
        )
    if report_path.exists():
        print(f"subagent 报告已写入：{report_path}")
    else:
        print("subagent 没有写出报告；请检查上面的日志", file=sys.stderr)
    return result.returncode


def _build_prompt(
    *,
    repo: Path,
    paper_arg: str,
    paper_path: Path,
    idea: str,
    task: str,
    report_path: Path,
) -> str:
    return f"""你是 LaCAM lifelong throughput CORAL 任务的聚焦实现 subagent。

仓库根目录：
{repo}

本任务作为依据的本地论文 brief：
{paper_arg}
解析后的路径：
{paper_path}

论文支撑的 idea：
{idea}

委托任务：
{task}

规则：
- 论文检索必须渐进式披露：先读较短的 {PAPER_CATEGORIES.as_posix()}，并把 idea 分类为 map/guidance-weight optimization、algorithm design 或 heuristic function design；然后读上面指定的 brief；只有需要章节、算法、设计模式或实验观察细节时，才读链接的原始 HTML/PDF 论文。较长的 {PAPER_README.as_posix()} 只用于局部检索相关条目，不要默认全文加载。
- 找出 brief、链接的论文文件，以及支撑 idea 的具体章节、算法、设计模式或实验观察。
- 只实现本任务需要的最小通用算法改动或诊断改动。
- 保持固定场景语义。不要硬编码 map、task、seed、output、schedule、CSV 或 trace 捷径。
- 运行必要的聚焦验证命令来建立信心。优先使用 build/tests、local probes 和 direct public-seed benchmark runs，避免范围过大且噪声高的命令。
- 永远不要运行 `coral eval --tune`。自测时直接用 `tools/run_symbotic_requested_grid.py` 跑 public tune seeds，输出到 `hl_agent/runs/<experiment>/runner`，这样结果不会注册为 CORAL attempt。
- diagnostic-only、counter-only、logging-only、refactor-only、runtime-only、no-op 或 parameter-path-not-hit 改动，不要运行 direct public-seed self-tests，也不要运行普通 `coral eval`。
- 任何 direct public-seed self-test 前，必须先证明 candidate 在 benchmark CLI defaults 下改变了 active algorithmic decision path。使用聚焦 local/single-seed probe 或 trace comparison，并报告行为变化信号：selected targets、move ordering、assignment rows、conflict/blocking counts、completed vector 或其他 decision-level metric。如果 completed vector 和关键 trace signals 与 parent 一致，标记为 no-op 或 instrumentation-only，不要跑 public-seed self-test 或 real。
- 每个已定形、能够 build/test 且输出 valid 的行为改变型 candidate，无论 local/public benchmark 改善、持平还是退化，都先单独 git commit 并运行普通 `coral eval -m "..."`，之后才能 retain、revert、丢弃或 pivot。不得因为自己的 benchmark/eval 结果不好而静默回退。回退到已有 official eval 的旧状态时引用已有 commit/attempt 结果，不为回退本身重复 benchmark/eval。
- 运行中持续写可恢复日志：开始前记录 parent commit、hypothesis、计划改动、风险和验证信号；每个重要命令后记录精确结果、artifact 路径、changed files、commit/eval attempt、决策和下一步。context 变大、可能自动 compact、重启或切换 agent 前先写 checkpoint；resume 后先读日志。
- 写一份简洁 markdown report 到：
  {report_path}

报告模板：
# 论文支撑 Subagent 报告

## 论文依据
- 主要类别：
- brief 文件：
- 链接的论文文件：
- 章节 / 算法 / 观察：
- 如何映射到当前代码库：

## 已完成工作
- 修改文件：
- 运行命令：
- eval 前的行为变化证据：
- 结果：

## 建议
- 保留 / 丢弃 / 需要后续：
- 建议 eval message：
"""


def _slug(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "-", text.strip().lower()).strip("-")
    return (slug or "subagent")[:60]


def _codex_cmd(prompt: str, model: str, effort: str) -> list[str]:
    return [
        "codex",
        "exec",
        prompt,
        "--dangerously-bypass-approvals-and-sandbox",
        "--model",
        model,
        "-c",
        f"model_reasoning_effort={effort!r}",
        "--json",
    ]


def _claude_cmd(prompt: str, model: str, effort: str) -> list[str]:
    return [
        "claude",
        "-p",
        prompt,
        "--model",
        model,
        "--effort",
        effort,
        "--permission-mode",
        "bypassPermissions",
        "--output-format",
        "stream-json",
        "--verbose",
    ]


def _normalize_runtime(value: str) -> str:
    normalized = value.strip().lower().replace("-", "_")
    if normalized == "claude_code":
        return "claude"
    if normalized in {"codex", "claude"}:
        return normalized
    raise ValueError(f"unsupported subagent runtime: {value}")


def _run_subagent(
    cmd: list[str], repo: Path, stdout_path: Path, stderr_path: Path
) -> subprocess.CompletedProcess[str]:
    with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
        return subprocess.run(
            cmd,
            cwd=repo,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            text=True,
            env=_clean_env(),
        )


def _looks_like_quota_error(stdout_path: Path, stderr_path: Path) -> bool:
    text = ""
    for path in (stdout_path, stderr_path):
        try:
            text += "\n" + path.read_text(errors="replace")
        except OSError:
            pass
    lowered = text.lower()
    return any(pattern in lowered for pattern in QUOTA_ERROR_PATTERNS)


def _is_paper_brief(repo: Path, paper_path: Path) -> bool:
    brief_root = (repo / PAPER_BRIEFS_ROOT).resolve()
    try:
        paper_path.relative_to(brief_root)
    except ValueError:
        return False
    return paper_path.suffix == ".md"


def _shell_quote(text: str) -> str:
    if re.fullmatch(r"[A-Za-z0-9_./:=+-]+", text):
        return text
    return "'" + text.replace("'", "'\"'\"'") + "'"


def _clean_env() -> dict[str, str]:
    env = os.environ.copy()
    env.pop("VIRTUAL_ENV", None)
    env.setdefault("LC_ALL", "C")
    return env


if __name__ == "__main__":
    raise SystemExit(main())
