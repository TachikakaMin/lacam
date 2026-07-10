#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/start_coral.sh codex [extra coral overrides...]
  scripts/start_coral.sh claude [extra coral overrides...]
  scripts/start_coral.sh resume codex RUN_ID [extra coral overrides...]
  scripts/start_coral.sh resume claude RUN_ID [extra coral overrides...]

Runtime presets:
  codex   -> agents.runtime=codex, agents.model=gpt-5.6-sol, reasoning=high
  claude  -> agents.runtime=claude_code, agents.model=fable, main policy=high

Subagent models are not pinned by this script. Agents choose them autonomously
based on the task, availability, latency, capability, quota, and rate limits.

Environment overrides:
  CORAL_AGENTS                 Agent count (default: 1)
  CORAL_SESSION                Run session (default: local)
EOF
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

mode="start"
if [[ "$1" == "resume" ]]; then
  mode="resume"
  shift
  if [[ $# -lt 2 ]]; then
    usage >&2
    exit 2
  fi
fi

runtime="$1"
shift

run_id=""
if [[ "$mode" == "resume" ]]; then
  run_id="$1"
  shift
fi

session="${CORAL_SESSION:-local}"
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_dir"

case "$runtime" in
  codex)
    count="${CORAL_AGENTS:-1}"
    preset=(
      agents.runtime=codex
      agents.model=gpt-5.6-sol
      agents.runtime_options.model_reasoning_effort=high
      agents.count="$count"
      run.session="$session"
    )
    ;;
  claude|claude_code)
    count="${CORAL_AGENTS:-1}"
    preset=(
      agents.runtime=claude_code
      agents.model=fable
      agents.runtime_options.model_reasoning_effort=high
      agents.count="$count"
      run.session="$session"
    )
    ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    echo "unknown runtime: $runtime" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ "$mode" == "resume" ]]; then
  exec coral resume --run "$run_id" "${preset[@]}" "$@"
fi

exec coral start -c "$task_dir/task.yaml" "${preset[@]}" "$@"
