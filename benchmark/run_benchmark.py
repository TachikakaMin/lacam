#!/usr/bin/env python3
"""Benchmark runner for baseline methods (design.md 8.1/8.3).

Methods:
  b4          — single-robot sequential simulation (Theorem 1); in-process.
  crest_base  — CREST executor with all constraint-release OFF
                (= MAPF-DECOMP-style decomposed execution).
  crest_full  — CREST with --STR --DW --GTR on.
  natcbs      — MAWR NAT-CBS (makespan-optimal), small instances only.

Unified metrics per row (rows.csv):
  instance, family, method, success, executed_makespan, weighted_soc,
  loaded_moves, free_moves, lift_drop, runtime_sec, status, raw

External binaries run through the ddtool micromamba env.  Converted inputs and
raw outputs are kept under <out>/work/ for auditability.
"""

import argparse
import csv
import ctypes
from decimal import Decimal, InvalidOperation
import fcntl
import hashlib
import socket
import json
import math
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ddbench.b4_baseline import B4Failure, solve_b4
from ddbench.converters import ConversionError, to_crest, to_mawr
from ddbench.instance import load_instance
from ddbench.validator import plan_cost, validate_plan

REPO = Path(__file__).resolve().parent.parent
BENCH = Path(__file__).resolve().parent
CREST_BIN = REPO / "baselines/CREST/build/CREST"
MAWR_BIN = REPO / "baselines/wh-rearrangement/build/MAWR"
CARRIER_BIN = REPO / "build/dd_benchmark"
MICROMAMBA = Path.home() / ".local/bin/micromamba"

BENCHMARK_TIERS = {
    "quick": {
        "suite": BENCH / "release_benchmark.json",
        "expected_cases": 77,
        "requires_review": False,
    },
    "full": {
        "suite": BENCH / "full_benchmark.json",
        "expected_cases": 509,
        "requires_review": True,
    },
}
FULL_ONLY_INSTANCES = (
    BENCH
    / "viz_web"
    / "warehouse_case_proposal"
    / "factorial_suite"
    / "instances"
)

FIELDS = [
    "instance", "family", "method", "success", "executed_makespan",
    "weighted_soc", "weighted_work_scaled",
    "loaded_moves", "free_moves", "lift_drop",
    "shelf_switches", "robot_utilization", "first_solution_ms",
    "first_solution_makespan", "first_solution_soc",
    "first_solution_work_scaled",
    "best_makespan", "best_soc", "best_work_scaled",
    "improvement_attempts", "improvement_candidates",
    "improvement_improvements", "improvement_generator_failures",
    "reference_checkpoint_hits", "reference_action_hints",
    "reference_suffix_attempts", "reference_suffix_accepted",
    "improvement_exit_reason",
    "reversals", "assignment_restarts", "assignment_second_solved",
    "assignment_improvements", "assignment_second_solution_ms",
    "assignment_first_soc", "assignment_second_soc",
    "assignment_first_makespan", "assignment_second_makespan",
    "upper_epoch_builds", "pair_cache_hits", "pair_cache_misses",
    "pair_rollout_steps", "pair_rollout_truncations",
    "pair_rollout_stalls", "tau_guide_changes_on_upper_move",
    "joint_task_nodes", "joint_task_edges", "joint_shared_effects",
    "joint_effect_conflicts", "joint_candidate_backtracks",
    "joint_paused_roots", "ready_task_count", "rho_repairs",
    "custody_continuations", "zero_empty_no_ready",
    "rewire_guidance_rebuilds",
    "tau_time_ms", "guidance_time_ms",
    "timed_transport_expansions", "timed_transport_frames",
    "timed_transport_time_ms", "owner_handoffs",
    "causal_waiting", "traffic_waiting",
    "deliverable_ms", "solver_runtime_ms",
    "plan_sha256",
    "runtime_sec", "status", "raw",
]

WORK_SCALE = 1_000_000
MAX_SOLVER_WEIGHT = 1_000_000
MAX_INT64 = (1 << 63) - 1


def _objective_weight_scaled(value):
    try:
        decimal_value = Decimal(str(value))
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"invalid objective weight: {value!r}") from exc
    if (not decimal_value.is_finite() or decimal_value < 0 or
            decimal_value > MAX_SOLVER_WEIGHT):
        raise ValueError(
            "objective weight must be finite, non-negative, and <= 1e6"
        )
    scaled = decimal_value * WORK_SCALE
    integral = scaled.to_integral_value()
    if scaled != integral:
        raise ValueError(
            "objective weight must be exactly representable at 1e-6 scale"
        )
    return int(integral)


def _reported_nonnegative_int(metrics, field):
    raw = metrics.get(field)
    if raw is None or re.fullmatch(r"(?:0|[1-9][0-9]*)", raw) is None:
        raise ValueError(
            f"solver omitted a valid non-negative integer {field}"
        )
    value = int(raw)
    if value > MAX_INT64:
        raise ValueError(f"solver reported out-of-range {field}")
    return value


def validate_carrier_work_metrics(metrics, cost, weights, mode):
    """Cross-check delivered-plan work in exact integer micro-units."""
    scaled_weights = tuple(
        _objective_weight_scaled(value) for value in weights
    )
    expected = (
        scaled_weights[0] * int(cost["loaded_moves"])
        + scaled_weights[1] * int(cost["free_moves"])
        + scaled_weights[2] * int(cost["lift_drop"])
        + scaled_weights[3] * int(cost["anon_moves"])
    )
    if expected > MAX_INT64:
        raise ValueError("authoritative fixed-point work exceeds int64")

    reported = _reported_nonnegative_int(
        metrics, "weighted_work_scaled"
    )
    if reported != expected:
        raise ValueError(
            "solver/Python work mismatch: "
            f"weighted_work_scaled {reported} != {expected}"
        )

    # B0/B1 predate v5 incumbent diagnostics, but their delivered-plan
    # accounting still crosses the same authoritative replay boundary.
    if mode == "lacam":
        best = _reported_nonnegative_int(metrics, "best_work_scaled")
        if best != expected:
            raise ValueError(
                "solver/Python work mismatch: "
                f"best_work_scaled {best} != {expected}"
            )
    return expected


def _sha256_file(path, label):
    # Keep procfs fd paths intact: resolve() follows a sealed memfd symlink to
    # the descriptive but non-openable "/memfd:... (deleted)" target.
    resolved = Path(path).expanduser().absolute()
    try:
        digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    except OSError as exc:
        raise ValueError(f"cannot hash {label} {resolved}: {exc}") from exc
    return resolved, digest


def _load_review_approval(review_approval):
    approval_path = Path(review_approval).expanduser().resolve()
    try:
        approval = json.loads(approval_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(
            f"cannot load independent review approval {approval_path}: {exc}"
        ) from exc
    if not isinstance(approval, dict):
        raise ValueError("independent review approval must be a JSON object")
    return approval_path, approval


def assert_approved_binary_unchanged(approval, carrier_bin=None):
    """Reconfirm that the approved executable bytes are still in place."""
    expected = approval.get("binary_sha256")
    if (not isinstance(expected, str) or
            re.fullmatch(r"[0-9a-f]{64}", expected) is None):
        raise ValueError(
            "review approval binary_sha256 must be 64 lowercase hex digits"
        )
    _, actual = _sha256_file(
        carrier_bin if carrier_bin is not None else CARRIER_BIN,
        "carrier binary",
    )
    if actual != expected:
        raise ValueError(
            "review approval binary hash does not match --carrier-bin"
        )
    return actual


def validate_full_review_approval(review_approval, carrier_bin=None):
    """Validate the independent full-suite approval and return its payload."""
    _, approval = _load_review_approval(review_approval)
    if approval.get("schema_version") != 2:
        raise ValueError("review approval schema_version must be 2")
    if approval.get("decision") != "APPROVE":
        raise ValueError("independent review decision must be APPROVE")
    if approval.get("reviewer_model") != "openai.gpt-5.6-sol":
        raise ValueError(
            "independent review must use openai.gpt-5.6-sol"
        )
    expected_hash = hashlib.sha256(
        BENCHMARK_TIERS["full"]["suite"].read_bytes()
    ).hexdigest()
    if approval.get("suite_definition_sha256") != expected_hash:
        raise ValueError(
            "review approval suite hash does not match full benchmark"
        )
    # Check the executable before the expensive corpus digest so stale or
    # malformed binary approvals fail immediately.
    assert_approved_binary_unchanged(approval, carrier_bin)
    expected_corpus_hash = full_corpus_sha256()
    if approval.get("full_corpus_sha256") != expected_corpus_hash:
        raise ValueError(
            "review approval corpus hash does not match full benchmark"
        )
    return approval


def resolve_benchmark_tier(
    tier, review_approval=None, carrier_bin=None
):
    """Resolve a fixed project benchmark tier.

    The development tier is the frozen 77-case release suite.  The full tier
    is intentionally gated by a suite-hash-bound independent review artifact,
    so it cannot be launched accidentally while implementation is in flight.
    """
    try:
        tier_definition = BENCHMARK_TIERS[tier]
    except KeyError as exc:
        raise ValueError(f"unknown benchmark tier: {tier}") from exc

    suite = tier_definition["suite"]
    if not tier_definition["requires_review"]:
        return suite
    if review_approval is None:
        raise ValueError(
            "full benchmark requires an independent review approval JSON"
        )

    validate_full_review_approval(
        review_approval, carrier_bin=carrier_bin
    )
    return suite


def guard_protected_suite(
    suite_path, review_approval=None, carrier_bin=None
):
    """Apply the full-tier review gate even for direct suite-config paths."""
    suite = Path(suite_path).expanduser().resolve()
    full_suite = BENCHMARK_TIERS["full"]["suite"].resolve()
    if suite == full_suite:
        return resolve_benchmark_tier(
            "full", review_approval=review_approval,
            carrier_bin=carrier_bin,
        )
    return suite


def _effective_storage_cells(ins):
    if ins.storage_cells is not None:
        return sorted(tuple(cell) for cell in ins.storage_cells)
    return sorted(
        (r, c)
        for r in range(ins.height)
        for c in range(ins.width)
        if not ins.grid[r][c]
    )


def _yaml_cpp_int(value):
    """Match yaml-cpp's base-detecting conversion for integer scalars."""
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    text = str(value).strip()
    match = re.fullmatch(r"([+-]?)(0[xX][0-9a-fA-F]+|[0-9]+)", text)
    if match is None:
        raise ValueError(f"invalid yaml-cpp integer scalar: {value!r}")
    sign = -1 if match.group(1) == "-" else 1
    digits = match.group(2)
    if digits.lower().startswith("0x"):
        base = 16
        digits = digits[2:]
    elif len(digits) > 1 and digits.startswith("0"):
        base = 8
    else:
        base = 10
    return sign * int(digits, base)


def _canonical_cell(cell, ins):
    row = _yaml_cpp_int(cell[0])
    col = _yaml_cpp_int(cell[1])
    if not (0 <= row < ins.height and 0 <= col < ins.width):
        raise ValueError(
            f"coordinate outside grid: {(row, col)} for "
            f"{ins.height}x{ins.width}"
        )
    return (row, col)


def semantic_case_fingerprint(path):
    """Hash normalized planning semantics, excluding YAML presentation."""
    ins = load_instance(path)
    payload = {
        "grid": ins.grid,
        "storage_cells": [
            list(cell) for cell in _effective_storage_cells(ins)
        ],
        # Robot order is semantic because robots are labeled.
        "robots": [
            list(_canonical_cell(cell, ins)) for cell in ins.robots
        ],
        # Shelves are anonymous except for target identities.
        "shelves": [
            list(cell)
            for cell in sorted(
                {_canonical_cell(cell, ins) for cell in ins.shelves}
            )
        ],
        "targets": [
            {
                "start": list(_canonical_cell(target.start, ins)),
                "eligible_goals": [
                    list(cell)
                    for cell in sorted(
                        {
                            _canonical_cell(goal, ins)
                            for goal in target.eligible_goals()
                        }
                    )
                ],
            }
            # Carrier ignores YAML target `id`, but target list order defines
            # the internal target indices and therefore remains significant.
            for target in ins.targets
        ],
    }
    text = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def corpus_sha256_for_cases(cases):
    """Digest normalized cases while preserving family and multiplicity."""
    entries = sorted(
        (family, semantic_case_fingerprint(path))
        for path, family in cases
    )
    text = json.dumps(entries, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def full_corpus_sha256():
    """Digest all 509 normalized cases while preserving multiplicity."""
    _, cases, _ = discover_suite_cases(
        BENCHMARK_TIERS["full"]["suite"]
    )
    return corpus_sha256_for_cases(cases)


_MEMFD_CREATE_SYSCALL = {
    "x86_64": 319,
    "aarch64": 279,
}
_MFD_CLOEXEC = 0x0001
_MFD_ALLOW_SEALING = 0x0002
_F_ADD_SEALS = 1033
_F_SEAL_ALL_WRITES = 0x0001 | 0x0002 | 0x0004 | 0x0008


def _sealed_memfd(name, data, executable=False):
    """Create a Linux sealed in-memory file and return ``(fd, path)``."""
    syscall_number = _MEMFD_CREATE_SYSCALL.get(platform.machine())
    if syscall_number is None:
        raise ValueError(
            "approved full benchmark requires Linux memfd sealing"
        )
    libc = ctypes.CDLL(None, use_errno=True)
    syscall = libc.syscall
    syscall.restype = ctypes.c_long
    fd = syscall(
        ctypes.c_long(syscall_number),
        ctypes.c_char_p(name.encode("utf-8")[:200]),
        ctypes.c_uint(_MFD_CLOEXEC | _MFD_ALLOW_SEALING),
    )
    if fd < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    try:
        view = memoryview(data)
        written = 0
        while written < len(view):
            count = os.write(fd, view[written:])
            if count <= 0:
                raise OSError("short write while creating sealed snapshot")
            written += count
        os.fchmod(fd, 0o500 if executable else 0o400)
        fcntl.fcntl(fd, _F_ADD_SEALS, _F_SEAL_ALL_WRITES)
        return fd, Path(f"/proc/{os.getpid()}/fd/{fd}")
    except Exception:
        os.close(fd)
        raise


class ApprovedExecutionSnapshot:
    """Sealed binary and YAML bytes used by an approved full run."""

    def __init__(self):
        self._fds = []
        self.binary_path = None
        self.binary_sha256 = ""
        self.corpus_sha256 = ""
        self.cases = []

    def add(self, name, data, executable=False):
        fd, path = _sealed_memfd(name, data, executable=executable)
        self._fds.append(fd)
        return path

    def close(self):
        while self._fds:
            os.close(self._fds.pop())

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def create_approved_execution_snapshot(cases, carrier_bin, approval):
    """Freeze exactly the approved executable and testcase semantics."""
    snapshot = ApprovedExecutionSnapshot()
    try:
        binary_data = Path(carrier_bin).read_bytes()
        snapshot.binary_sha256 = hashlib.sha256(binary_data).hexdigest()
        if snapshot.binary_sha256 != approval.get("binary_sha256"):
            raise ValueError(
                "review approval binary hash changed before snapshot"
            )
        snapshot.binary_path = snapshot.add(
            "approved-dd-benchmark", binary_data, executable=True
        )

        semantic_cases = []
        for index, (path, family) in enumerate(cases):
            original = Path(path)
            data = original.read_bytes()
            sealed_path = snapshot.add(
                f"approved-case-{index}-{original.name}", data
            )
            snapshot.cases.append(
                (sealed_path, family, original.stem)
            )
            semantic_cases.append((sealed_path, family))
        snapshot.corpus_sha256 = corpus_sha256_for_cases(
            semantic_cases
        )
        if snapshot.corpus_sha256 != approval.get(
                "full_corpus_sha256"):
            raise ValueError(
                "review approval corpus hash changed before snapshot"
            )
        return snapshot
    except Exception:
        snapshot.close()
        raise


def guard_full_only_cases(
    cases, review_approval=None, carrier_bin=None
):
    """Require review for any copied, renamed, or partial full-only corpus."""
    protected_hashes = {
        semantic_case_fingerprint(path)
        for path in FULL_ONLY_INSTANCES.glob("*.yaml")
        if path.is_file()
    }
    matched = [
        Path(path)
        for path, _ in cases
        if semantic_case_fingerprint(path) in protected_hashes
    ]
    if matched:
        resolve_benchmark_tier(
            "full", review_approval=review_approval,
            carrier_bin=carrier_bin,
        )
    elif review_approval is not None:
        raise ValueError(
            "review approval is only valid when running full-only cases"
        )
    return matched


def provenance_info(carrier_bin=None):
    """R6 (debug.md §10): commit, binary hash and host for timing.json."""
    binary_path = Path(
        CARRIER_BIN if carrier_bin is None else carrier_bin
    )
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True,
            cwd=REPO, timeout=10).stdout.strip() or "unknown"
    except Exception:  # noqa: BLE001
        commit = "unknown"
    try:
        binary_sha = hashlib.sha256(binary_path.read_bytes()).hexdigest()
    except OSError:
        binary_sha = ""
    return {"git_commit": commit, "binary_path": str(binary_path),
            "binary_sha256": binary_sha,
            "host": socket.gethostname()}


def ensure_out_dir(out, force):
    """R6: refuse to silently overwrite an existing result directory."""
    out = Path(out)
    if (out / "rows.csv").exists() and not force:
        raise SystemExit(
            f"out dir {out} already holds rows.csv; results are audit "
            f"artifacts (protocol §11.1(8)) - pick a new directory or pass "
            f"--force to overwrite explicitly")
    out.mkdir(parents=True, exist_ok=True)


def write_rows(path, rows):
    """Write repository-auditable CSV with stable LF line endings."""
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=FIELDS, restval="", lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(
            {
                key: value.rstrip() if isinstance(value, str) else value
                for key, value in row.items()
            }
            for row in rows
        )


def load_suite_definition(path):
    """Load a protected benchmark-suite definition."""
    path = Path(path).expanduser().resolve()
    try:
        raw = path.read_bytes()
        definition = json.loads(raw.decode("utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load suite definition {path}: {exc}") from exc
    if definition.get("schema_version") != 1:
        raise ValueError("suite definition schema_version must be 1")
    if not isinstance(definition.get("name"), str) or not definition["name"]:
        raise ValueError("suite definition requires a non-empty name")
    if not isinstance(definition.get("protocol"), dict):
        raise ValueError("suite definition requires a protocol object")
    groups = definition.get("groups")
    if not isinstance(groups, list) or not groups:
        raise ValueError("suite definition requires at least one group")
    definition["_definition_path"] = str(path)
    definition["_definition_sha256"] = hashlib.sha256(raw).hexdigest()
    return definition


def discover_suite_cases(path):
    """Resolve ``(YAML path, family)`` pairs from a suite definition.

    Expected group cardinalities and unique instance stems are enforced so a
    missing generated corpus or an accidental duplicate cannot silently alter
    the protected benchmark.
    """
    definition = load_suite_definition(path)
    definition_path = Path(definition["_definition_path"])
    cases = []
    group_counts = {}
    seen_paths = set()
    seen_names = set()

    for group in definition["groups"]:
        if not isinstance(group, dict):
            raise ValueError("suite group must be an object")
        name = group.get("name")
        root_value = group.get("root")
        pattern = group.get("pattern")
        expected = group.get("expected_cases")
        if not isinstance(name, str) or not name:
            raise ValueError("suite group requires a non-empty name")
        if name in group_counts:
            raise ValueError(f"duplicate suite group name: {name}")
        if not isinstance(root_value, str) or not root_value:
            raise ValueError(f"suite group {name} requires a root")
        if not isinstance(pattern, str) or not pattern:
            raise ValueError(f"suite group {name} requires a pattern")
        if not isinstance(expected, int) or expected <= 0:
            raise ValueError(
                f"suite group {name} expected_cases must be positive"
            )

        root = (definition_path.parent / root_value).resolve()
        if not root.is_dir():
            raise ValueError(f"suite group {name} root does not exist: {root}")
        files = sorted(path for path in root.glob(pattern) if path.is_file())
        if len(files) != expected:
            raise ValueError(
                f"suite group {name} has {len(files)} cases, expected "
                f"{expected}"
            )
        group_counts[name] = len(files)

        fixed_family = group.get("family")
        if fixed_family is not None and (
            not isinstance(fixed_family, str) or not fixed_family
        ):
            raise ValueError(f"suite group {name} has an invalid family")
        for case_path in files:
            resolved = case_path.resolve()
            if resolved in seen_paths:
                raise ValueError(f"duplicate suite path: {resolved}")
            if case_path.stem in seen_names:
                raise ValueError(
                    f"duplicate suite instance name: {case_path.stem}"
                )
            seen_paths.add(resolved)
            seen_names.add(case_path.stem)
            cases.append(
                (resolved, fixed_family or case_path.parent.name)
            )

    return definition, cases, group_counts


def validate_deliverable_ms(metrics, timeout):
    """Machine-check the strict solver deliverable deadline on successes."""
    raw = metrics.get("deliverable_ms")
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("missing or invalid deliverable_ms") from exc
    limit_ms = float(timeout) * 1000.0
    if not math.isfinite(value) or value < 0 or value > limit_ms:
        raise ValueError(
            f"deliverable_ms={raw} exceeds strict limit {limit_ms}"
        )
    return value


def validate_solver_runtime_ms(metrics, timeout):
    """Machine-check the measured C++ solve-call return deadline."""
    raw = metrics.get("runtime_ms")
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("missing or invalid runtime_ms") from exc
    limit_ms = float(timeout) * 1000.0
    if not math.isfinite(value) or value < 0 or value > limit_ms:
        raise ValueError(
            f"runtime_ms={raw} exceeds strict limit {limit_ms}"
        )
    return value


def run_external(cmd, timeout):
    # run binaries directly with the conda env's libs: the micromamba wrapper
    # serializes on a lock under heavy parallelism and skews timing.
    env = dict(
        os.environ,
        LD_LIBRARY_PATH=str(Path.home() / "micromamba/envs/ddtool/lib"),
    )
    full = [str(c) for c in cmd]
    t0 = time.time()
    proc = subprocess.Popen(
        full, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        env=env, start_new_session=True,
    )
    try:
        out, err = proc.communicate(timeout=timeout)
        return proc.returncode, out, err, time.time() - t0, "ok"
    except subprocess.TimeoutExpired:
        import signal
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        proc.communicate()
        return -1, "", "", timeout, "timeout"


def _blank_extra():
    return dict(shelf_switches="", robot_utilization="", reversals="",
                first_solution_ms="")


def row_b4(ins, name, family, timeout, weights=(1.0, 1.0, 1.0, 1.0)):
    t0 = time.time()
    try:
        plan = solve_b4(ins)
    except B4Failure as e:
        return dict(instance=name, family=family, method="b4", success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(time.time() - t0, 3),
                    status="failed", raw=str(e))
    runtime = time.time() - t0
    ok, errs, _ = validate_plan(ins, plan)
    if not ok:
        return dict(instance=name, family=family, method="b4", success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(runtime, 3),
                    status="invalid_plan", raw=";".join(errs)[:200])
    c = plan_cost(ins, plan, *weights)
    return dict(instance=name, family=family, method="b4", success=1,
                executed_makespan=c["executed_makespan"],
                weighted_soc=c["weighted_soc"], loaded_moves=c["loaded_moves"],
                free_moves=c["free_moves"], lift_drop=c["lift_drop"],
                shelf_switches=c["shelf_switches"],
                reversals=c["reversals"],
                robot_utilization=round(c["robot_utilization"], 4),
                first_solution_ms="",
                runtime_sec=round(runtime, 3), status="ok", raw="")


CREST_RE = re.compile(
    r"sum_of_cost : (\S+) makespan : (\S+) agent_travel_time : (\S+) "
    r"pickup_count : (\S+) rearrange_shelf_num : (\S+) "
    r"pickup_dropoff_overhead : (\S+)"
)


def row_crest(ins, name, family, work, timeout, full_release, subopt=1.6):
    method = "crest_full" if full_release else "crest_base"
    mp = work / f"{name}.crest.map"
    sp = work / f"{name}.crest.scen"
    try:
        to_crest(ins, mp, sp)
    except ConversionError as e:
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=0,
                    status="conversion_error", raw=str(e))
    flag = "true" if full_release else "false"
    rc, out, err, rt, status = run_external(
        [CREST_BIN, f"--suboptimality={subopt}", "-m", mp, "-a", sp,
         "-k", str(len(ins.robots)), f"--STR={flag}", f"--DW={flag}",
         f"--GTR={flag}", "--overhead=false", "-t", str(timeout)],
        timeout + 30,
    )
    (work / f"{name}.{method}.out").write_text(out + "\n--stderr--\n" + err)
    m = CREST_RE.search(out)
    if status != "ok" or rc != 0 or not m:
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                    status=status if status != "ok" else f"rc={rc}",
                    raw=(out + err)[-200:].replace("\n", " "))
    soc, mk, travel, pickups, _, overhead = m.groups()
    # CREST loaded_moves = soc - travel - overhead (agent timesteps split)
    return dict(instance=name, family=family, method=method, success=1,
                executed_makespan=int(mk),
                weighted_soc=int(soc),
                loaded_moves=int(soc) - int(travel) - int(overhead),
                free_moves=int(travel), lift_drop=int(overhead),
                **_blank_extra(), runtime_sec=round(rt, 3), status="ok",
                raw=m.group(0)[:200])


def row_natcbs(ins, name, family, work, timeout):
    mp = work / f"{name}.mawr.map"
    sp = work / f"{name}.mawr.scen"
    to_mawr(ins, mp, sp)
    out_csv = work / f"{name}.mawr.csv"
    if out_csv.exists():
        out_csv.unlink()
    out_csv.touch()
    rc, out, err, rt, status = run_external(
        [MAWR_BIN, "-m", mp, "-s", sp, "-a", "NATCBS",
         "-t", str(int(timeout)), "-o", out_csv],
        timeout + 30,
    )
    (work / f"{name}.natcbs.out").write_text(out + "\n--stderr--\n" + err)
    mk = ""
    st = status
    if status == "ok":
        lines = out_csv.read_text().strip().splitlines()
        if len(lines) >= 2:
            parts = lines[-1].split(";")
            val = parts[2]
            if val == "TLR":
                st = "timeout"
            elif val in ("NF", "IVP"):
                st = "failed"
            else:
                mk = int(val)
                st = "ok"
        else:
            st = f"rc={rc}"
    success = 1 if mk != "" else 0
    return dict(instance=name, family=family, method="natcbs", success=success,
                executed_makespan=mk, weighted_soc="", loaded_moves="",
                free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                status=st, raw=(out + err)[-150:].replace("\n", " "))


def parse_carrier_plan(path, allow_empty=False):
    path = Path(path)
    if not path.is_file():
        raise ValueError("solver reported success without a plan file")
    plan = []
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        joint = []
        for token in line.split(";"):
            parts = token.split()
            if parts == ["w"]:
                joint.append(("wait",))
            elif len(parts) == 3 and parts[0] == "m":
                try:
                    cell = (int(parts[1]), int(parts[2]))
                except ValueError as exc:
                    raise ValueError(
                        f"line {line_no}: invalid move token {token!r}"
                    ) from exc
                joint.append(("move", cell))
            elif parts == ["l"]:
                joint.append(("lift",))
            elif parts == ["d"]:
                joint.append(("drop",))
            else:
                raise ValueError(
                    f"line {line_no}: invalid action token {token!r}"
                )
        plan.append(joint)
    if not plan and not allow_empty:
        raise ValueError("solver reported success with an empty plan")
    return plan


def row_carrier(ins, path, name, family, work, timeout, mode="lacam",
                env=None, weights=(1.0, 1.0, 1.0, 1.0),
                carrier_bin=None):
    """Carrier-LaCAM (C++): plan re-validated by the authoritative Python
    two-deck validator; unified metrics via plan_cost (same as b4)."""
    from ddbench.validator import apply_joint_action, initial_state, is_goal

    method = {"lacam": "carrier", "b0": "carrier_b0",
              "b1": "carrier_b1"}[mode]
    plan_out = work / f"{name}.{method}.plan"
    if plan_out.exists():
        plan_out.unlink()
    t0 = time.time()
    env = dict(os.environ) if env is None else dict(env)
    executable = Path(
        CARRIER_BIN if carrier_bin is None else carrier_bin
    )
    for key, value in zip(
        ("DD_ALPHA", "DD_BETA", "DD_GAMMA", "DD_DELTA"), weights
    ):
        env[key] = str(value)
    try:
        p = subprocess.run(
            [str(executable), str(path), str(timeout), str(plan_out), "0",
             mode],
            capture_output=True, text=True, timeout=timeout, env=env,
        )
        status = "ok"
    except subprocess.TimeoutExpired:
        p = None
        status = "timeout"
    rt = time.time() - t0
    metrics = {}
    if p is not None:
        for line in p.stdout.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                metrics[k.strip()] = v.strip()
    if (status != "ok" or p.returncode != 0 or
            metrics.get("solved") != "1"):
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                    status="timeout" if status != "ok" or
                    metrics.get("timed_out") == "1" else "failed",
                    raw=(p.stdout + p.stderr)[-150:].replace("\n", " ")
                    if p else "")
    try:
        validate_deliverable_ms(metrics, timeout)
        validate_solver_runtime_ms(metrics, timeout)
    except ValueError as e:
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(),
                    runtime_sec=round(rt, 3), status="deadline_violation",
                    raw=str(e))
    # authoritative re-validation
    try:
        plan = parse_carrier_plan(plan_out, allow_empty=True)
        s = initial_state(ins)
        for joint in plan:
            s = apply_joint_action(ins, s, joint)
        if not is_goal(ins, s):
            raise ValueError("final state is not a goal")
        c = plan_cost(ins, plan, *weights)
        try:
            reported_makespan = int(metrics["makespan"])
        except (KeyError, ValueError) as exc:
            raise ValueError("solver omitted a valid integer makespan") from exc
        if reported_makespan != c["executed_makespan"]:
            raise ValueError(
                "solver/Python makespan mismatch: "
                f"{reported_makespan} != {c['executed_makespan']}"
            )
        work_scaled = validate_carrier_work_metrics(
            metrics, c, weights, mode
        )
    except Exception as e:  # noqa: BLE001
        return dict(instance=name, family=family, method=method, success=0,
                    executed_makespan="", weighted_soc="", loaded_moves="",
                    free_moves="", lift_drop="", **_blank_extra(), runtime_sec=round(rt, 3),
                    status="invalid_plan", raw=str(e)[:200])
    return dict(instance=name, family=family, method=method, success=1,
                executed_makespan=c["executed_makespan"],
                weighted_soc=c["weighted_soc"],
                weighted_work_scaled=work_scaled,
                loaded_moves=c["loaded_moves"],
                free_moves=c["free_moves"], lift_drop=c["lift_drop"],
                shelf_switches=c["shelf_switches"],
                reversals=c["reversals"],
                robot_utilization=round(c["robot_utilization"], 4),
                first_solution_ms=metrics.get("first_solution_ms", ""),
                first_solution_makespan=metrics.get(
                    "first_solution_makespan", ""
                ),
                first_solution_soc=metrics.get("first_solution_soc", ""),
                first_solution_work_scaled=metrics.get(
                    "first_solution_work_scaled", ""
                ),
                best_makespan=metrics.get("best_makespan", ""),
                best_soc=metrics.get("best_soc", ""),
                best_work_scaled=metrics.get("best_work_scaled", ""),
                improvement_attempts=metrics.get(
                    "improvement_attempts", ""
                ),
                improvement_candidates=metrics.get(
                    "improvement_candidates", ""
                ),
                improvement_improvements=metrics.get(
                    "improvement_improvements", ""
                ),
                improvement_generator_failures=metrics.get(
                    "improvement_generator_failures", ""
                ),
                reference_checkpoint_hits=metrics.get(
                    "reference_checkpoint_hits", ""
                ),
                reference_action_hints=metrics.get(
                    "reference_action_hints", ""
                ),
                reference_suffix_attempts=metrics.get(
                    "reference_suffix_attempts", ""
                ),
                reference_suffix_accepted=metrics.get(
                    "reference_suffix_accepted", ""
                ),
                improvement_exit_reason=metrics.get(
                    "improvement_exit_reason", ""
                ),
                assignment_restarts=metrics.get("assignment_restarts", ""),
                assignment_second_solved=metrics.get(
                    "assignment_second_solved", ""
                ),
                assignment_improvements=metrics.get(
                    "assignment_improvements", ""
                ),
                assignment_second_solution_ms=metrics.get(
                    "assignment_second_solution_ms", ""
                ),
                assignment_first_soc=metrics.get(
                    "assignment_first_soc", ""
                ),
                assignment_second_soc=metrics.get(
                    "assignment_second_soc", ""
                ),
                assignment_first_makespan=metrics.get(
                    "assignment_first_makespan", ""
                ),
                assignment_second_makespan=metrics.get(
                    "assignment_second_makespan", ""
                ),
                upper_epoch_builds=metrics.get("upper_epoch_builds", ""),
                pair_cache_hits=metrics.get("pair_cache_hits", ""),
                pair_cache_misses=metrics.get("pair_cache_misses", ""),
                pair_rollout_steps=metrics.get("pair_rollout_steps", ""),
                pair_rollout_truncations=metrics.get(
                    "pair_rollout_truncations", ""
                ),
                pair_rollout_stalls=metrics.get(
                    "pair_rollout_stalls", ""
                ),
                tau_guide_changes_on_upper_move=metrics.get(
                    "tau_guide_changes_on_upper_move", ""
                ),
                joint_task_nodes=metrics.get("joint_task_nodes", ""),
                joint_task_edges=metrics.get("joint_task_edges", ""),
                joint_shared_effects=metrics.get(
                    "joint_shared_effects", ""
                ),
                joint_effect_conflicts=metrics.get(
                    "joint_effect_conflicts", ""
                ),
                joint_candidate_backtracks=metrics.get(
                    "joint_candidate_backtracks", ""
                ),
                joint_paused_roots=metrics.get(
                    "joint_paused_roots", ""
                ),
                ready_task_count=metrics.get("ready_task_count", ""),
                rho_repairs=metrics.get("rho_repairs", ""),
                custody_continuations=metrics.get(
                    "custody_continuations", ""
                ),
                zero_empty_no_ready=metrics.get(
                    "zero_empty_no_ready", ""
                ),
                rewire_guidance_rebuilds=metrics.get(
                    "rewire_guidance_rebuilds", ""
                ),
                tau_time_ms=metrics.get("tau_time_ms", ""),
                guidance_time_ms=metrics.get("guidance_time_ms", ""),
                timed_transport_expansions=metrics.get(
                    "timed_transport_expansions", ""
                ),
                timed_transport_frames=metrics.get(
                    "timed_transport_frames", ""
                ),
                timed_transport_time_ms=metrics.get(
                    "timed_transport_time_ms", ""
                ),
                owner_handoffs=metrics.get("owner_handoffs", ""),
                causal_waiting=metrics.get("causal_waiting", ""),
                traffic_waiting=metrics.get("traffic_waiting", ""),
                deliverable_ms=metrics.get("deliverable_ms", ""),
                solver_runtime_ms=metrics.get("runtime_ms", ""),
                plan_sha256=hashlib.sha256(
                    plan_out.read_bytes()).hexdigest(),
                runtime_sec=round(rt, 3), status="ok", raw="")


def run_one(task):
    """Top-level worker over one immutable case/method task."""
    (path, name, family, method, work, timeout, natcbs_max, subopt,
     weights, carrier_bin) = task
    ins = load_instance(path)
    work = Path(work)
    if method == "b4":
        return row_b4(ins, name, family, timeout, weights)
    if method == "carrier":
        return row_carrier(
            ins, path, name, family, work, timeout, "lacam",
            weights=weights, carrier_bin=carrier_bin,
        )
    if method == "carrier_b0":
        return row_carrier(
            ins, path, name, family, work, timeout, "b0",
            weights=weights, carrier_bin=carrier_bin,
        )
    if method == "carrier_b1":
        return row_carrier(
            ins, path, name, family, work, timeout, "b1",
            weights=weights, carrier_bin=carrier_bin,
        )
    if method == "crest_base":
        return row_crest(ins, name, family, work, timeout, False, subopt)
    if method == "crest_full":
        return row_crest(ins, name, family, work, timeout, True, subopt)
    if method == "natcbs":
        if ins.height * ins.width > natcbs_max:
            return dict(instance=name, family=family, method="natcbs",
                        success=0, executed_makespan="", weighted_soc="",
                        loaded_moves="", free_moves="", lift_drop="",
                        **_blank_extra(), runtime_sec=0, status="skipped_too_large", raw="")
        return row_natcbs(ins, name, family, work, timeout)
    raise SystemExit(f"unknown method {method}")


def main():
    global CARRIER_BIN

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--instances",
        help="instance root containing FAMILY/*.yaml (default: instances)",
    )
    ap.add_argument(
        "--suite-config",
        help="protected JSON suite definition; mutually exclusive with "
        "--instances",
    )
    ap.add_argument(
        "--benchmark-tier",
        choices=sorted(BENCHMARK_TIERS),
        help="fixed project tier: quick=77 development cases; "
        "full=509 post-review cases",
    )
    ap.add_argument(
        "--review-approval",
        type=Path,
        help="APPROVE JSON required by --benchmark-tier full",
    )
    ap.add_argument("--out-dir", default="results")
    ap.add_argument("--methods", nargs="+",
                    help="methods to run; suite configs provide a fixed list")
    ap.add_argument(
        "--timeout", type=float, default=10,
        help="per-run solver deadline in seconds (fixed protocol: 10)",
    )
    ap.add_argument("--natcbs-max-cells", type=int, default=150,
                    help="skip natcbs on instances larger than this")
    ap.add_argument(
        "--jobs", type=int,
        help="parallel worker processes; suite configs provide a fixed value",
    )
    ap.add_argument("--suboptimality", type=float, default=1.6,
                    help="CREST ECBS suboptimality bound")
    ap.add_argument("--force", action="store_true",
                    help="allow overwriting an existing result directory")
    ap.add_argument(
        "--carrier-bin", default=str(CARRIER_BIN),
        help="carrier executable, used for isolated compile-time ablations",
    )
    ap.add_argument(
        "--weights", type=float, nargs=4,
        metavar=("ALPHA", "BETA", "GAMMA", "DELTA"),
        default=(1.0, 1.0, 1.0, 1.0),
        help="carrier/B4 objective weights; defaults to the unit main table",
    )
    args = ap.parse_args()
    CARRIER_BIN = Path(args.carrier_bin).expanduser().resolve()
    selected_sources = sum(
        value is not None
        for value in (
            args.instances,
            args.suite_config,
            args.benchmark_tier,
        )
    )
    if selected_sources > 1:
        ap.error(
            "--instances, --suite-config, and --benchmark-tier are "
            "mutually exclusive"
        )

    tier_name = args.benchmark_tier
    if tier_name is not None:
        try:
            args.suite_config = str(
                resolve_benchmark_tier(
                    tier_name,
                    review_approval=args.review_approval,
                    carrier_bin=CARRIER_BIN,
                )
            )
        except ValueError as exc:
            ap.error(str(exc))
    elif args.suite_config is not None:
        try:
            args.suite_config = str(
                guard_protected_suite(
                    args.suite_config,
                    review_approval=args.review_approval,
                    carrier_bin=CARRIER_BIN,
                )
            )
        except ValueError as exc:
            ap.error(str(exc))
    elif args.review_approval is not None and args.instances is None:
        ap.error("--review-approval requires --benchmark-tier full")

    suite_timing = None
    if args.suite_config:
        try:
            definition, cases, group_counts = discover_suite_cases(
                args.suite_config
            )
        except ValueError as exc:
            ap.error(str(exc))
        protocol = definition["protocol"]
        required_protocol = {
            "methods", "timeout_sec", "jobs", "solver_seed",
            "objective_weights", "following",
        }
        if set(protocol) != required_protocol:
            ap.error(
                "suite protocol keys must be exactly "
                f"{sorted(required_protocol)}"
            )
        methods = list(args.methods or protocol["methods"])
        jobs = args.jobs if args.jobs is not None else int(protocol["jobs"])
        weights = tuple(args.weights)
        if methods != list(protocol["methods"]):
            ap.error(
                f"suite {definition['name']} fixes methods to "
                f"{protocol['methods']}"
            )
        if float(args.timeout) != float(protocol["timeout_sec"]):
            ap.error(
                f"suite {definition['name']} fixes --timeout to "
                f"{protocol['timeout_sec']}"
            )
        if jobs != int(protocol["jobs"]):
            ap.error(
                f"suite {definition['name']} fixes --jobs to "
                f"{protocol['jobs']}"
            )
        if weights != tuple(float(x) for x in protocol["objective_weights"]):
            ap.error(
                f"suite {definition['name']} fixes --weights to "
                f"{protocol['objective_weights']}"
            )
        if protocol["solver_seed"] != 0:
            ap.error("run_benchmark supports only solver_seed=0")
        if protocol["following"] != "allowed":
            ap.error("run_benchmark supports only following=allowed")
        definition_path = Path(definition["_definition_path"])
        suite_timing = {
            "name": definition["name"],
            "definition_path": str(definition_path),
            "definition_sha256": definition["_definition_sha256"],
            "groups": group_counts,
        }
        if tier_name is not None:
            expected_cases = BENCHMARK_TIERS[tier_name]["expected_cases"]
            if len(cases) != expected_cases:
                ap.error(
                    f"{tier_name} benchmark has {len(cases)} cases, "
                    f"expected {expected_cases}"
                )
            suite_timing["tier"] = tier_name
            suite_timing["review_approval"] = (
                str(args.review_approval.resolve())
                if args.review_approval is not None
                else None
            )
            if tier_name == "full":
                try:
                    _, quick_cases, _ = discover_suite_cases(
                        BENCHMARK_TIERS["quick"]["suite"]
                    )
                except ValueError as exc:
                    ap.error(str(exc))
                quick_names = {path.stem for path, _ in quick_cases}
                full_names = {path.stem for path, _ in cases}
                if not quick_names < full_names:
                    ap.error(
                        "full benchmark must be a strict superset of quick"
                    )
    else:
        root = Path(args.instances or "instances")
        files = sorted(root.glob("*/*.yaml"))
        cases = [(path.resolve(), path.parent.name) for path in files]
        methods = list(
            args.methods
            or ["b4", "crest_base", "crest_full", "natcbs"]
        )
        jobs = args.jobs if args.jobs is not None else 1
        weights = tuple(args.weights)

    try:
        full_only_cases = guard_full_only_cases(
            cases, review_approval=args.review_approval,
            carrier_bin=CARRIER_BIN,
        )
    except ValueError as exc:
        ap.error(str(exc))
    if suite_timing is not None:
        suite_timing["full_only_case_count"] = len(full_only_cases)

    if any(m in {"carrier", "carrier_b0", "carrier_b1"}
           for m in methods):
        if not CARRIER_BIN.is_file() or not os.access(CARRIER_BIN, os.X_OK):
            ap.error(f"--carrier-bin is not executable: {CARRIER_BIN}")
    if (weights != (1.0, 1.0, 1.0, 1.0) and
            any(m in {"crest_base", "crest_full", "natcbs"}
                for m in methods)):
        ap.error("non-unit --weights cannot be mixed with native-objective "
                 "external methods")

    active_full_approval = None
    execution_snapshot = None
    active_carrier_bin = CARRIER_BIN
    full_suite_run = (
        args.suite_config is not None and
        Path(args.suite_config).resolve() ==
        BENCHMARK_TIERS["full"]["suite"].resolve()
    )
    if full_only_cases:
        try:
            _, active_full_approval = _load_review_approval(
                args.review_approval
            )
            # Reconfirm immediately before creating work or dispatching jobs.
            assert_approved_binary_unchanged(
                active_full_approval, CARRIER_BIN
            )
            if full_suite_run:
                if (suite_timing is None or
                        suite_timing["definition_sha256"] !=
                        active_full_approval[
                            "suite_definition_sha256"]):
                    raise ValueError(
                        "full suite changed after review validation"
                    )
                execution_snapshot = (
                    create_approved_execution_snapshot(
                        cases, CARRIER_BIN, active_full_approval
                    )
                )
                active_carrier_bin = execution_snapshot.binary_path
        except ValueError as exc:
            ap.error(str(exc))

    out = Path(args.out_dir)
    ensure_out_dir(out, args.force)  # R6: no silent overwrites
    work = out / "work"
    work.mkdir(parents=True, exist_ok=True)

    if not cases:
        print("no instances found", file=sys.stderr)
        sys.exit(1)
    if execution_snapshot is not None:
        case_specs = execution_snapshot.cases
    else:
        case_specs = [
            (Path(path), family, Path(path).stem)
            for path, family in cases
        ]
    tasks = [
        (str(path), name, family, method, str(work), args.timeout,
         args.natcbs_max_cells, args.suboptimality, weights,
         str(active_carrier_bin))
        for path, family, name in case_specs
        for method in methods
    ]

    t_start = time.time()
    rows = []
    if jobs <= 1:
        for t in tasks:
            r = run_one(t)
            rows.append(r)
            print(f"[{len(rows)}/{len(tasks)}] {r['instance']} [{r['method']}]"
                  f" -> {r['status']} mk={r['executed_makespan']}"
                  f" t={r['runtime_sec']}s", flush=True)
    else:
        from concurrent.futures import ProcessPoolExecutor, as_completed
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            futs = {ex.submit(run_one, t): t for t in tasks}
            for fut in as_completed(futs):
                r = fut.result()
                rows.append(r)
                print(f"[{len(rows)}/{len(tasks)}] {r['instance']}"
                      f" [{r['method']}] -> {r['status']}"
                      f" mk={r['executed_makespan']}"
                      f" t={r['runtime_sec']}s", flush=True)
    wall = time.time() - t_start

    rows.sort(key=lambda r: (r["instance"], r["method"]))

    # summary + timing
    from collections import defaultdict
    agg = defaultdict(lambda: [0, 0, 0.0])
    for r in rows:
        agg[r["method"]][0] += r["success"] if isinstance(r["success"], int) else int(r["success"])
        agg[r["method"]][1] += 1
        agg[r["method"]][2] += float(r["runtime_sec"] or 0)
    print("\n=== success rate (method: solved/total, total solver time) ===",
          flush=True)
    summary = {}
    for m, (s, n, tt) in sorted(agg.items()):
        print(f"{m}: {s}/{n}  solver_time_sum={tt:.1f}s", flush=True)
        summary[m] = {"solved": s, "total": n, "solver_time_sum_sec": round(tt, 1)}
    if active_full_approval is not None:
        try:
            assert_approved_binary_unchanged(
                active_full_approval, active_carrier_bin
            )
            if execution_snapshot is not None:
                snapshot_cases = [
                    (path, family)
                    for path, family, _ in execution_snapshot.cases
                ]
                if (corpus_sha256_for_cases(snapshot_cases) !=
                        active_full_approval["full_corpus_sha256"]):
                    raise ValueError(
                        "sealed full corpus changed before publication"
                    )
        except ValueError as exc:
            ap.error(str(exc))
    provenance = provenance_info(active_carrier_bin)
    if execution_snapshot is not None:
        provenance["binary_path"] = str(CARRIER_BIN)
        provenance["execution_snapshot"] = "sealed_linux_memfd"
    if (active_full_approval is not None and
            provenance["binary_sha256"] !=
            active_full_approval["binary_sha256"]):
        ap.error(
            "provenance binary hash changed after full review validation"
        )
    timing = {
        "wall_time_sec": round(wall, 1),
        "jobs": jobs,
        "n_tasks": len(tasks),
        "timeout_per_run_sec": args.timeout,
        "solver_seed": 0,
        "objective_weights": {
            "alpha": weights[0],
            "beta": weights[1],
            "gamma": weights[2],
            "delta": weights[3],
        },
        "following": "allowed",
        "provenance": provenance,
        "methods": summary,
    }
    if suite_timing is not None:
        timing["suite"] = suite_timing
    out.mkdir(parents=True, exist_ok=True)
    write_rows(out / "rows.csv", rows)
    (out / "timing.json").write_text(json.dumps(timing, indent=2))
    if execution_snapshot is not None:
        execution_snapshot.close()
    print(f"wall_time={wall:.1f}s jobs={jobs}", flush=True)
    print(f"rows written to {out / 'rows.csv'}", flush=True)


if __name__ == "__main__":
    main()
