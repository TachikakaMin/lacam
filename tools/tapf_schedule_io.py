#!/usr/bin/env python3
from __future__ import annotations

import struct
from pathlib import Path
from typing import Any

import yaml


MAGIC = b"TAPFSCH1"


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _binary_path(schedule_yaml: Path, metadata: dict[str, Any]) -> Path:
    raw = metadata.get("path")
    if not raw:
        raise ValueError(f"schedule_binary.path missing in {schedule_yaml}")
    path = Path(raw)
    if path.is_absolute() or path.exists():
        return path
    return schedule_yaml.parent / path


def read_sparse_binary(path: Path) -> tuple[int, list[list[dict[str, int]]]]:
    data = path.read_bytes()
    offset = 0

    def take(size: int) -> bytes:
        nonlocal offset
        chunk = data[offset : offset + size]
        if len(chunk) != size:
            raise ValueError(f"truncated schedule binary: {path}")
        offset += size
        return chunk

    magic = take(len(MAGIC))
    if magic != MAGIC:
        raise ValueError(f"bad schedule binary magic in {path!s}: {magic!r}")
    agents, makespan = struct.unpack_from("<II", take(8))
    schedule = []
    for _ in range(agents):
        (count,) = struct.unpack_from("<I", take(4))
        path_entries = []
        for _ in range(count):
            t, x, y = struct.unpack_from("<III", take(12))
            path_entries.append({"t": int(t), "x": int(x), "y": int(y)})
        schedule.append(path_entries)
    if offset != len(data):
        raise ValueError(f"trailing bytes in schedule binary: {path}")
    return int(makespan), schedule


def load_schedule(schedule_yaml: Path) -> dict[str, Any]:
    data = load_yaml(schedule_yaml) or {}
    binary = data.get("schedule_binary")
    if binary:
        makespan, paths = read_sparse_binary(_binary_path(schedule_yaml, binary))
        data.setdefault("statistics", {})["makespan"] = makespan
        data["schedule"] = {f"agent{i}": path for i, path in enumerate(paths)}
    return data
