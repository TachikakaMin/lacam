#!/usr/bin/env python3
"""Export the verified LDraw parts and reverse-derived build plan for the web."""
from __future__ import annotations

import json
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

import generate_brougham_ldraw as model
import render_brougham_ldraw as ldraw

BASE = Path(__file__).resolve().parent

INSTALL_STEP = {
    13: 14,
    21: 27, 22: 27, 23: 27, 24: 27, 25: 27, 26: 27,
    28: 33, 29: 33, 30: 33, 31: 33, 32: 33,
    36: 37,
    39: 40,
    45: 46,
}

HEX = {
    0: "#24282e", 1: "#0055bf", 2: "#237841", 4: "#c91a09",
    6: "#a65318", 7: "#9ba19d", 14: "#f2cd37", 15: "#f4f4f0",
    16: "#777777", 46: "#ffd92f", 47: "#b8e4f2", 70: "#6c3a20",
    71: "#a0a5a9", 72: "#6c6e68", 320: "#720e18",
}


def transformed_faces(part):
    name = "4493c01" if part["part"] == "4493c01pb04" else part["part"]
    matrix = np.array(part["matrix"], dtype=float).reshape(3, 3)
    offset = np.array(part["position"], dtype=float)
    faces = []
    ldraw.expand(name + ".dat", matrix, offset, part["ldraw_color"], faces)
    return faces


def world_point(v):
    # One world unit per stud; align wheel/hoof contact (LDraw Y=79) to Y=0.
    return [round(v[0] / 20, 4), round((79 - v[1]) / 20, 4),
            round(-v[2] / 20, 4)]


def export_geometry():
    exported = []
    for part in model.parts:
        groups = defaultdict(list)
        for face, color in transformed_faces(part):
            triangles = [face] if len(face) == 3 else [
                face[[0, 1, 2]], face[[0, 2, 3]]]
            for tri in triangles:
                for vertex in tri:
                    groups[color].extend(world_point(vertex))
        exported.append({
            "id": part["id"],
            "part": part["part"],
            "step": part["step"],
            "installStep": INSTALL_STEP.get(part["step"], part["step"]),
            "groups": [{
                "color": HEX.get(color, "#888888"),
                "transparent": color == 47,
                "positions": positions,
            } for color, positions in sorted(groups.items())],
        })
    text = "window.BROUGHAM_GEOMETRY=" + json.dumps(
        exported, separators=(",", ":")) + ";\n"
    (BASE / "brougham_exact_geometry.js").write_text(text)
    return exported


def derive_connections():
    """Encode the physical final-assembly graph visible in the 48 PDF steps."""
    by_id = {part["id"]: part for part in model.parts}
    edges = []

    def link(child, parent, kind):
        assert child in by_id and parent in by_id
        edges.append({"child": child, "parent": parent, "kind": kind})

    # Rear shell and rear window.
    link("s02-p002", "s01-p001", "stud")
    link("s03-p003", "s02-p002", "stud")
    link("s03-p004", "s02-p002", "stud")
    link("s04-p005", "s03-p003", "stud")
    link("s04-p006", "s03-p003", "stud")
    link("s04-p007", "s03-p004", "stud")
    link("s05-p008", "s05-p009", "glazing-in-frame")
    link("s05-p009", "s04-p005", "stud")
    link("s06-p010", "s04-p006", "stud")
    link("s06-p011", "s04-p007", "stud")

    # Front chassis, front window and coachman's deck/seats.
    link("s07-p012", "s01-p001", "edge-stud")
    link("s08-p013", "s07-p012", "stud")
    link("s08-p014", "s07-p012", "stud")
    link("s09-p015", "s08-p013", "stud")
    link("s09-p016", "s08-p014", "stud")
    link("s10-p017", "s07-p012", "stud")
    link("s11-p018", "s10-p017", "stud")
    link("s11-p019", "s10-p017", "stud")
    link("s12-p020", "s11-p018", "stud")
    link("s12-p020", "s11-p019", "stud")
    link("s13-p021", "s13-p022", "glazing-in-frame")
    link("s13-p022", "s11-p018", "stud")
    link("s13-p022", "s11-p019", "stud")
    link("s15-p023", "s12-p020", "stud")
    link("s15-p024", "s15-p023", "stud")
    link("s16-p025", "s15-p024", "stud")
    link("s17-p026", "s16-p025", "stud")
    link("s17-p027", "s16-p025", "stud")
    link("s18-p028", "s17-p026", "stud")
    link("s18-p029", "s17-p027", "stud")
    link("s19-p030", "s17-p026", "stud")
    link("s19-p031", "s17-p027", "stud")
    link("s20-p032", "s19-p030", "stud")
    link("s20-p033", "s19-p031", "stud")

    # Step 21-26 driver box, attached to the front deck at step 27.
    link("s21-p034", "s12-p020", "front-box-lower-mount")
    link("s22-p035", "s21-p034", "stud")
    link("s22-p036", "s21-p034", "stud")
    link("s22-p037", "s21-p034", "stud")
    link("s22-p038", "s21-p034", "stud")
    for child in ("s23-p039",):
        link(child, "s22-p035", "stud")
        link(child, "s22-p037", "stud")
    link("s24-p040", "s23-p039", "stud")
    link("s25-p041", "s24-p040", "side-stud")
    link("s25-p042", "s25-p041", "side-stud")
    link("s26-p043", "s23-p039", "stud")
    link("s26-p044", "s23-p039", "stud")

    # Front and rear running gear.
    link("s28-p045", "s07-p012", "front-axle-module-install")
    link("s28-p046", "s28-p045", "wheel-pin")
    link("s28-p047", "s28-p045", "wheel-pin")
    link("s29-p048", "s28-p045", "stud")
    link("s30-p049", "s29-p048", "clip-handle")
    link("s31-p050", "s30-p049", "clip-bar")
    link("s32-p051", "s29-p048", "stud-pin")
    link("s32-p051", "s07-p012", "turntable-pin")
    link("s34-p052", "s01-p001", "underside-stud")
    link("s35-p053", "s34-p052", "underside-stud")
    link("s35-p054", "s35-p053", "wheel-pin")
    link("s35-p055", "s35-p053", "wheel-pin")

    # Cabin interior, lower doors and side window frames.
    link("s36-p056", "s04-p005", "interior-seat-mount")
    link("s36-p057", "s36-p056", "stud")
    link("s38-p058", "s07-p012", "door-stud")
    link("s38-p058", "s06-p010", "side-wall")
    link("s38-p059", "s07-p012", "door-stud")
    link("s38-p059", "s06-p011", "side-wall")
    link("s39-p060", "s39-p061", "glazing-in-frame")
    link("s39-p061", "s38-p058", "stud")
    link("s39-p062", "s39-p063", "glazing-in-frame")
    link("s39-p063", "s38-p059", "stud")

    # Roof rail, cornice, lamp brackets, lanterns and roof plate.
    link("s41-p064", "s39-p061", "stud")
    link("s41-p065", "s39-p063", "stud")
    link("s41-p066", "s05-p009", "stud")
    link("s41-p067", "s13-p022", "stud")
    link("s41-p068", "s13-p022", "stud")
    link("s42-p069", "s41-p067", "stud")
    link("s42-p069", "s41-p068", "stud")
    link("s42-p070", "s42-p069", "stud")
    link("s42-p071", "s42-p069", "stud")
    link("s43-p072", "s42-p069", "stud")
    link("s43-p073", "s43-p072", "stud")
    link("s44-p074", "s42-p070", "bracket-clip")
    link("s44-p075", "s42-p071", "bracket-clip")
    link("s45-p077", "s44-p074", "lantern-hook")
    link("s45-p076", "s45-p077", "insert")
    link("s45-p079", "s44-p075", "lantern-hook")
    link("s45-p078", "s45-p079", "insert")
    link("s47-p080", "s41-p064", "roof-stud")
    link("s47-p080", "s41-p065", "roof-stud")

    # Horse is inserted between the two integral shafts; the white brick is
    # attached to the horse's own two back studs, as shown in PDF step 48.
    link("s48-p081", "s31-p050", "harness-between-shafts")
    link("s48-p082", "s48-p081", "horse-back-studs")
    link("s48-p082", "s31-p050", "hitch-crossplate-stud")
    return edges


def topological_order(edges):
    ids = [part["id"] for part in model.parts]
    parents = defaultdict(set)
    children = defaultdict(set)
    for edge in edges:
        parents[edge["child"]].add(edge["parent"])
        children[edge["parent"]].add(edge["child"])
    indegree = {part_id: len(parents[part_id]) for part_id in ids}
    by_id = {part["id"]: part for part in model.parts}
    ready = [part_id for part_id in ids if indegree[part_id] == 0]
    ready.sort(key=lambda part_id: (
        INSTALL_STEP.get(by_id[part_id]["step"], by_id[part_id]["step"]),
        by_id[part_id]["step"], part_id))
    ordered = []
    while ready:
        part_id = ready.pop(0)
        ordered.append(part_id)
        for child in sorted(children[part_id]):
            indegree[child] -= 1
            if indegree[child] == 0:
                ready.append(child)
                ready.sort(key=lambda node: (
                    INSTALL_STEP.get(by_id[node]["step"], by_id[node]["step"]),
                    by_id[node]["step"], node))
    if len(ordered) != len(ids):
        raise RuntimeError("physical connection graph contains a cycle")

    roots = [part_id for part_id in ids if not parents[part_id]]
    if roots != ["s01-p001"]:
        raise RuntimeError("expected the step-1 4x4 plate as the only root: " +
                           repr(roots))
    reached = set()
    queue = deque(roots)
    while queue:
        node = queue.popleft()
        if node in reached:
            continue
        reached.add(node)
        queue.extend(children[node])
    if reached != set(ids):
        raise RuntimeError("connection graph is not a single component")
    return ordered, parents


def validate_connection_distances(edges):
    boxes = {}
    for part in model.parts:
        vertices = np.concatenate([face[0] for face in transformed_faces(part)])
        boxes[part["id"]] = (vertices.min(axis=0), vertices.max(axis=0))

    relaxed = {
        "front-axle-module-install": 20.1,
        "interior-seat-mount": 10.1,
        "harness-between-shafts": 20.1,
    }
    measurements = []
    for edge in edges:
        child_lo, child_hi = boxes[edge["child"]]
        parent_lo, parent_hi = boxes[edge["parent"]]
        delta = np.maximum(
            0, np.maximum(child_lo - parent_hi, parent_lo - child_hi))
        distance = float(np.linalg.norm(delta))
        limit = relaxed.get(edge["kind"], 4.1)
        if distance > limit:
            raise RuntimeError(
                "disconnected physical edge %s -> %s (%s): %.3f LDU > %.3f"
                % (edge["child"], edge["parent"], edge["kind"],
                   distance, limit))
        measurements.append({
            "child": edge["child"],
            "parent": edge["parent"],
            "distanceLdu": round(distance, 4),
            "limitLdu": limit,
        })
    return measurements


ROBOT_RADIUS = .34
GROUND_LANE_Z = 8.5
SIDE_STAGING_X = 4.25
MODEL_GROUND_BOUNDS = {
    "xMin": -2.35,
    "xMax": 2.35,
    "zMin": -11.75,
    "zMax": 4.985,
}


def route_for(part, robot, assembly_height):
    """Plan a grounded mobile base plus a telescoping manipulator trajectory."""
    park = [-8.75 + robot * 1.25, 0.0, 10.0]
    depot = [park[0], 0.0, GROUND_LANE_Z]
    target = world_point(np.array(part["position"], dtype=float))
    side = -1 if target[0] < 0 else 1
    if abs(target[0]) < 1e-9:
        side = -1 if robot % 2 == 0 else 1
    side_x = side * SIDE_STAGING_X
    rear_lane = [side_x, 0.0, GROUND_LANE_Z]
    staging = [side_x, 0.0, target[2]]
    lift_y = max(1.35, target[1] + 1.5, assembly_height + .8)
    carry_height = .72

    base_path = [
        park,
        depot,
        rear_lane,
        staging,
        staging,
        staging,
        staging,
        rear_lane,
        depot,
        park,
    ]
    part_path = [
        [park[0], carry_height, park[2]],
        [depot[0], carry_height, depot[2]],
        [rear_lane[0], carry_height, rear_lane[2]],
        [staging[0], carry_height, staging[2]],
        [staging[0], lift_y, staging[2]],
        [target[0], lift_y, target[2]],
        target,
        target,
        target,
        target,
    ]
    return {
        "basePath": base_path,
        "partPath": part_path,
        "target": target,
        "pickupWaypoint": 1,
        "depositWaypoint": 6,
        "assemblyHeightBefore": round(assembly_height, 4),
        "liftClearanceY": round(lift_y, 4),
    }


def validate_ground_routes(tasks):
    """Reject flying bases and paths crossing the completed model footprint."""
    expanded = {
        "xMin": MODEL_GROUND_BOUNDS["xMin"] - ROBOT_RADIUS,
        "xMax": MODEL_GROUND_BOUNDS["xMax"] + ROBOT_RADIUS,
        "zMin": MODEL_GROUND_BOUNDS["zMin"] - ROBOT_RADIUS,
        "zMax": MODEL_GROUND_BOUNDS["zMax"] + ROBOT_RADIUS,
    }
    park_positions = [
        np.array([-8.75 + robot * 1.25, 0.0, 10.0])
        for robot in range(8)
    ]
    minimum_model_clearance = float("inf")
    minimum_robot_clearance = float("inf")
    for task in tasks:
        path = task["basePath"]
        if any(abs(point[1]) > 1e-9 for point in path):
            raise RuntimeError("mobile base leaves the ground: " +
                               task["partId"])
        for a, b in zip(path, path[1:]):
            a = np.array(a, dtype=float)
            b = np.array(b, dtype=float)
            for alpha in np.linspace(0.0, 1.0, 41):
                point = a + (b - a) * alpha
                dx = max(expanded["xMin"] - point[0], 0.0,
                         point[0] - expanded["xMax"])
                dz = max(expanded["zMin"] - point[2], 0.0,
                         point[2] - expanded["zMax"])
                clearance = float(np.hypot(dx, dz))
                minimum_model_clearance = min(
                    minimum_model_clearance, clearance)
                if dx == 0.0 and dz == 0.0:
                    raise RuntimeError(
                        "ground route enters model footprint: " +
                        task["partId"])
                for robot, parked in enumerate(park_positions):
                    if robot == task["robot"]:
                        continue
                    separation = float(np.linalg.norm(
                        point[[0, 2]] - parked[[0, 2]]))
                    minimum_robot_clearance = min(
                        minimum_robot_clearance,
                        separation - 2 * ROBOT_RADIUS)
                    if separation < 2 * ROBOT_RADIUS:
                        raise RuntimeError(
                            "moving base collides with parked robot: " +
                            task["partId"])
        if task["liftClearanceY"] < task["assemblyHeightBefore"] + .79:
            raise RuntimeError("manipulator lift clearance is insufficient: " +
                               task["partId"])
    return {
        "allBaseWaypointsGrounded": True,
        "modelFootprintClearance": round(minimum_model_clearance, 4),
        "parkedRobotClearance": round(minimum_robot_clearance, 4),
    }


def export_plan():
    connections = derive_connections()
    ordered_ids, parents = topological_order(connections)
    distance_checks = validate_connection_distances(connections)
    by_id = {part["id"]: part for part in model.parts}
    tasks = []
    assembly_height = 0.0
    for rank, part_id in enumerate(ordered_ids):
        part = by_id[part_id]
        install = INSTALL_STEP.get(part["step"], part["step"])
        robot = rank % 8
        route = route_for(part, robot, assembly_height)
        task = {
            "rank": rank + 1,
            "reverseRank": len(ordered_ids) - rank,
            "partId": part["id"],
            "part": part["part"],
            "color": part["color"],
            "pdfStep": part["step"],
            "installStep": install,
            "robot": robot,
            "dependsOn": sorted(parents[part_id]),
            "startTick": rank * 9,
            "finishTick": (rank + 1) * 9,
            **route,
        }
        tasks.append(task)
        part_vertices = np.concatenate(
            [face[0] for face in transformed_faces(part)])
        part_height = float(np.max((79.0 - part_vertices[:, 1]) / 20.0))
        assembly_height = max(assembly_height, part_height)
    ground_checks = validate_ground_routes(tasks)
    plan = {
        "source": "MOC-194603_brickbicycle_brougham.pdf",
        "method": "final physical connection graph -> reverse topological "
                  "disassembly -> reversed robot build order",
        "partCount": len(tasks),
        "instructionSteps": 48,
        "robots": 8,
        "connections": connections,
        "reverseDisassembly": list(reversed(ordered_ids)),
        "verification": {
            "singleConnectedComponent": True,
            "acyclic": True,
            "connectionDistanceChecks": distance_checks,
            "singleMovingRobot": True,
            "robotRobotCollisionFree": True,
            "allRobotBasesGrounded": True,
            "groundRouteChecks": ground_checks,
            "routeStrategy": "serialized ground perimeter lanes with "
                             "stationary telescoping-manipulator placement",
        },
        "tasks": tasks,
    }
    payload = json.dumps(plan, indent=2) + "\n"
    (BASE / "brougham_exact_build_plan.json").write_text(payload)
    (BASE / "brougham_exact_build_plan.js").write_text(
        "window.BROUGHAM_BUILD_PLAN=" + json.dumps(
            plan, separators=(",", ":")) + ";\n")
    return plan


def main():
    model.audit()
    geometry = export_geometry()
    plan = export_plan()
    assert len(geometry) == plan["partCount"] == 82
    assert len({task["partId"] for task in plan["tasks"]}) == 82
    assert len(plan["reverseDisassembly"]) == 82
    assert all(task["finishTick"] <= plan["tasks"][i + 1]["startTick"]
               for i, task in enumerate(plan["tasks"][:-1]))
    print("exported 82 exact parts, connected DAG, and collision-free routes")


if __name__ == "__main__":
    main()
