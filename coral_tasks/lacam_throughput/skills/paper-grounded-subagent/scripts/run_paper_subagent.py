#!/usr/bin/env python3
"""Launch a paper-grounded Codex subagent for LaCAM throughput work."""

from __future__ import annotations

import argparse
import datetime as dt
import os
import re
import subprocess
import sys
from pathlib import Path


SUBAGENT_MODEL = "gpt-5.3-codex-spark"
SUBAGENT_EFFORT = "high"
PAPER_BRIEFS_ROOT = Path("coral_tasks/lacam_throughput/papers/briefs")
PAPER_README = Path("coral_tasks/lacam_throughput/papers/README.md")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a Codex subagent grounded in a local paper-library source."
    )
    parser.add_argument("--paper", required=True, help="Local paper brief path that grounds the idea.")
    parser.add_argument("--idea", required=True, help="One concise paper-grounded hypothesis.")
    parser.add_argument("--task", required=True, help="Concrete work delegated to the subagent.")
    parser.add_argument(
        "--out-dir",
        default="hl_agent/runs/paper_grounded_subagents",
        help="Directory for prompts, logs, and reports.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Write prompt/report paths and print the Codex command without launching it.",
    )
    args = parser.parse_args()

    repo = Path.cwd()
    paper_path = (repo / args.paper).resolve()
    if not paper_path.exists():
        print(f"paper source not found: {args.paper}", file=sys.stderr)
        return 2
    if not _is_paper_brief(repo, paper_path):
        print(
            "paper source must be a local paper brief under "
            f"{PAPER_BRIEFS_ROOT.as_posix()}; start from "
            f"{PAPER_README.as_posix()} and pass the selected brief",
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
    stdout_path = run_dir / "codex.stdout.jsonl"
    stderr_path = run_dir / "codex.stderr.txt"

    prompt = _build_prompt(
        repo=repo,
        paper_arg=args.paper,
        paper_path=paper_path,
        idea=args.idea,
        task=args.task,
        report_path=report_path,
    )
    prompt_path.write_text(prompt)

    cmd = [
        "codex",
        "exec",
        prompt,
        "--dangerously-bypass-approvals-and-sandbox",
        "--model",
        SUBAGENT_MODEL,
        "-c",
        f"model_reasoning_effort={SUBAGENT_EFFORT!r}",
        "--json",
    ]

    print(f"paper source: {args.paper}")
    print(f"prompt: {prompt_path}")
    print(f"report: {report_path}")
    print(f"stdout log: {stdout_path}")
    print(f"stderr log: {stderr_path}")
    print("command: " + " ".join(_shell_quote(part) for part in cmd))
    if args.dry_run:
        return 0

    with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
        result = subprocess.run(
            cmd,
            cwd=repo,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            text=True,
            env=_clean_env(),
        )

    print(f"subagent exit code: {result.returncode}")
    if report_path.exists():
        print(f"subagent report written: {report_path}")
    else:
        print("subagent report was not written; inspect logs above", file=sys.stderr)
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
    return f"""You are a focused implementation subagent for the LaCAM lifelong throughput CORAL task.

Repository root:
{repo}

Local paper brief that grounds this task:
{paper_arg}
Resolved path:
{paper_path}

Paper-grounded idea:
{idea}

Delegated task:
{task}

Rules:
- Paper search is progressive disclosure: first read
  {PAPER_README.as_posix()}, then the cited brief above, and only then the
  linked original HTML/PDF paper if section, algorithm, design-pattern, or
  empirical-observation detail is needed.
- Identify the brief, linked paper file, and concrete section, algorithm,
  design pattern, or empirical observation that motivates the idea.
- Implement only the smallest general algorithmic or diagnostic change needed
  for this task.
- Preserve fixed scenario semantics. Do not hard-code map, task, seed, output,
  schedule, CSV, or trace shortcuts.
- Run focused verification commands needed for confidence. Prefer build/tests
  and tune/local diagnostics over broad noisy commands.
- Run `coral eval --tune` or normal `coral eval` when the delegated task asks
  you to score the candidate. Include the paper grounding in `coral eval -m`.
- Write a concise markdown report to:
  {report_path}

Report template:
# Paper-Grounded Subagent Report

## Paper Grounding
- Brief file:
- Linked paper file:
- Section / algorithm / observation:
- How it maps to this codebase:

## Work Completed
- Files changed:
- Commands run:
- Results:

## Recommendation
- Keep / discard / needs follow-up:
- Suggested eval message:
"""


def _slug(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "-", text.strip().lower()).strip("-")
    return (slug or "subagent")[:60]


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
