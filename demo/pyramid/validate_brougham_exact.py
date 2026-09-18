#!/usr/bin/env python3
"""End-to-end acceptance checks for the exact MOC-194603 reconstruction."""
from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

import numpy as np

import build_brougham_exact_web as web
import generate_brougham_ldraw as model

BASE = Path(__file__).resolve().parent


def vertices(part):
    return np.concatenate([face[0] for face in web.transformed_faces(part)])


def box_distance(a, b):
    av, bv = vertices(a), vertices(b)
    alo, ahi = av.min(axis=0), av.max(axis=0)
    blo, bhi = bv.min(axis=0), bv.max(axis=0)
    delta = np.maximum(0, np.maximum(alo - bhi, blo - ahi))
    return float(np.linalg.norm(delta))


def main():
    model.audit()
    assert len(model.parts) == 82
    assert len({part["id"] for part in model.parts}) == 82

    bom = json.loads((BASE / "brougham_exact_bom.json").read_text())
    steps = json.loads((BASE / "brougham_exact_steps.json").read_text())
    expected = Counter((part["part"], part["color"])
                       for part in bom["parts"] for _ in range(part["count"]))
    actual = Counter((part["part"], part["color"]) for part in model.parts)
    assert expected == actual
    assert bom["total_parts"] == 82
    assert len(steps["steps"]) == 48

    by_id = {part["id"]: part for part in model.parts}
    all_vertices = {}
    for part in model.parts:
        matrix = np.array(part["matrix"], dtype=float).reshape(3, 3)
        assert np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-7)
        assert np.isclose(abs(np.linalg.det(matrix)), 1.0, atol=1e-7)
        all_vertices[part["id"]] = vertices(part)
        assert len(all_vertices[part["id"]]) > 0

    connections = web.derive_connections()
    ordered, parents = web.topological_order(connections)
    checks = web.validate_connection_distances(connections)
    assert len(ordered) == 82
    assert len(checks) == len(connections)
    assert ordered[0] == "s01-p001"

    plan = json.loads((BASE / "brougham_exact_build_plan.json").read_text())
    model_json = json.loads((BASE / "brougham_exact_model.json").read_text())
    assert model_json["model_status"] == "verified-against-pdf-key-steps"
    assert model_json["geometry_fallbacks"][0]["part"] == "4493c01pb04"
    assert plan["partCount"] == len(plan["tasks"]) == 82
    assert [task["partId"] for task in plan["tasks"]] == ordered
    assert plan["reverseDisassembly"] == list(reversed(ordered))
    assert plan["verification"]["singleConnectedComponent"]
    assert plan["verification"]["acyclic"]
    assert plan["verification"]["robotRobotCollisionFree"]
    for index, task in enumerate(plan["tasks"]):
        assert task["dependsOn"] == sorted(parents[task["partId"]])
        assert len(task["basePath"]) == len(task["partPath"]) == 10
        assert task["basePath"][0] == task["basePath"][-1]
        assert all(point[1] == 0.0 for point in task["basePath"])
        assert 0 <= task["pickupWaypoint"] < task["depositWaypoint"] \
            < len(task["partPath"])
        deposit = task["partPath"][task["depositWaypoint"]]
        assert deposit == task["target"]
        assert task["liftClearanceY"] >= task["assemblyHeightBefore"] + .79
        if index:
            assert plan["tasks"][index - 1]["finishTick"] <= task["startTick"]

    # Four wheel bottoms and horse hooves share the same ground plane.
    wheel_ids = ("s28-p046", "s28-p047", "s35-p054", "s35-p055")
    wheel_bottoms = [all_vertices[part_id][:, 1].max()
                     for part_id in wheel_ids]
    assert max(wheel_bottoms) - min(wheel_bottoms) < 1e-6
    horse_bottom = all_vertices["s48-p081"][:, 1].max()
    assert abs(horse_bottom - wheel_bottoms[0]) < 1e-6

    # Load-bearing/attachment landmarks visible in PDF steps 32, 47 and 48.
    assert box_distance(by_id["s32-p051"], by_id["s07-p012"]) == 0
    assert box_distance(by_id["s47-p080"], by_id["s41-p064"]) == 0
    assert box_distance(by_id["s47-p080"], by_id["s41-p065"]) == 0
    assert box_distance(by_id["s48-p082"], by_id["s48-p081"]) == 0

    html = (BASE / "brougham.html").read_text()
    app = (BASE / "brougham_exact_app.js").read_text()
    assert "brougham_exact_geometry.js" in html
    assert "brougham_exact_build_plan.js" in html
    assert "brougham_exact_app.js" in html
    assert html.count("v=exact-5") == 4
    assert "plan_brougham.js" not in html
    assert "app.js?v=pdf" not in html
    assert "let progress = buildMode ? 0 : plan.partCount" in app
    assert "let yaw = 2.25" in app
    for filename in (
        "brougham_exact_model.ldr",
        "brougham_exact_model.json",
        "brougham_exact_model.png",
        "brougham_exact_geometry.js",
        "brougham_exact_build_plan.json",
        "brougham_exact_build_plan.js",
    ):
        assert (BASE / filename).is_file(), filename

    print("MOC-194603 exact acceptance: PASS")
    print("82 parts; 48 PDF steps; %d physical edges; 82 robot tasks"
          % len(connections))


if __name__ == "__main__":
    main()
