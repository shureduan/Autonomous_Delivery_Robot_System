from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import pandas as pd


# =========================
# Data structures
# =========================

@dataclass
class Node:
    node_id: str
    node_type: str
    floor: int
    x: float
    y: float
    z: float

    @property
    def pos(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass
class EdgeDef:
    start_id: str
    end_id: str
    edge_type: str      # fixed / theta / transfer
    map_name: str       # ground / platform / floor2 / elevator / ramp2 ...


@dataclass
class EdgeCacheRecord:
    start_id: str
    end_id: str
    edge_type: str
    map_name: str
    distance: Optional[float]
    waypoints: List[List[float]]
    floor: Optional[int]
    status: str         # ok / pending_theta / missing_fixed / error
    note: str = ""


# =========================
# Loaders
# =========================

def load_nodes(nodes_csv: str) -> Dict[str, Node]:
    df = pd.read_csv(nodes_csv)

    required_cols = {"id", "type", "floor", "x", "y", "z"}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"scene_nodes.csv 缺少列: {missing}")

    nodes: Dict[str, Node] = {}
    for _, row in df.iterrows():
        node = Node(
            node_id=str(row["id"]).strip(),
            node_type=str(row["type"]).strip(),
            floor=int(row["floor"]),
            x=float(row["x"]),
            y=float(row["y"]),
            z=float(row["z"]),
        )
        nodes[node.node_id] = node

    return nodes


def load_edge_defs(edge_file: str) -> List[EdgeDef]:
    path = Path(edge_file)
    if path.suffix.lower() == ".csv":
        df = pd.read_csv(path)
    elif path.suffix.lower() in [".xlsx", ".xls"]:
        df = pd.read_excel(path)
    else:
        raise ValueError(f"不支持的边定义文件格式: {path.suffix}")

    required_cols = {"start_id", "end_id", "edge_type", "map_name"}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"edge_definition 缺少列: {missing}")

    edges: List[EdgeDef] = []
    for _, row in df.iterrows():
        edges.append(
            EdgeDef(
                start_id=str(row["start_id"]).strip(),
                end_id=str(row["end_id"]).strip(),
                edge_type=str(row["edge_type"]).strip().lower(),
                map_name=str(row["map_name"]).strip(),
            )
        )
    return edges


# =========================
# Utilities
# =========================

def polyline_length(points: List[Tuple[float, float, float]]) -> float:
    if len(points) < 2:
        return 0.0

    total = 0.0
    for i in range(len(points) - 1):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[i + 1]
        total += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return total


def to_waypoint_list(points: List[Tuple[float, float, float]]) -> List[List[float]]:
    return [[float(x), float(y), float(z)] for x, y, z in points]


def infer_floor_for_edge(start_node: Node, end_node: Node, edge_type: str) -> Optional[int]:
    if edge_type == "transfer":
        return None
    if start_node.floor == end_node.floor:
        return start_node.floor
    return None


# =========================
# Fixed route adapter
# =========================
# 你后面把自己已有固定路线导出的格式接到这里。
# 目前先假设 fixed_routes.json 长这样：
#
# {
#   "Ground_Depot->Ground_Ramp2_Bottom": [[x,y,z], [x,y,z], ...],
#   "Ground_Ramp2_Bottom->Platform_Ramp2_Top": [[x,y,z], [x,y,z], ...]
# }
#
# 如果你目前不是这个格式，后面只改这里一个函数就行。

def load_fixed_routes(fixed_routes_json: Optional[str]) -> Dict[str, List[List[float]]]:
    if fixed_routes_json is None:
        return {}

    path = Path(fixed_routes_json)
    if not path.exists():
        print(f"[WARN] fixed route 文件不存在: {fixed_routes_json}")
        return {}

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    return data


def build_fixed_record(
    edge: EdgeDef,
    start_node: Node,
    end_node: Node,
    fixed_routes: Dict[str, List[List[float]]]
) -> EdgeCacheRecord:
    key_forward = f"{edge.start_id}->{edge.end_id}"
    key_reverse = f"{edge.end_id}->{edge.start_id}"

    if key_forward in fixed_routes:
        raw_points = fixed_routes[key_forward]
        points = [tuple(map(float, p)) for p in raw_points]
    elif key_reverse in fixed_routes:
        raw_points = fixed_routes[key_reverse]
        points = [tuple(map(float, p)) for p in raw_points[::-1]]
    else:
        return EdgeCacheRecord(
            start_id=edge.start_id,
            end_id=edge.end_id,
            edge_type=edge.edge_type,
            map_name=edge.map_name,
            distance=None,
            waypoints=[],
            floor=infer_floor_for_edge(start_node, end_node, edge.edge_type),
            status="missing_fixed",
            note="未找到对应 fixed 路线"
        )

    return EdgeCacheRecord(
        start_id=edge.start_id,
        end_id=edge.end_id,
        edge_type=edge.edge_type,
        map_name=edge.map_name,
        distance=polyline_length(points),
        waypoints=to_waypoint_list(points),
        floor=infer_floor_for_edge(start_node, end_node, edge.edge_type),
        status="ok",
        note=""
    )


# =========================
# Transfer edge builder
# =========================

def build_transfer_record(edge: EdgeDef, start_node: Node, end_node: Node) -> EdgeCacheRecord:
    points = [start_node.pos, end_node.pos]

    return EdgeCacheRecord(
        start_id=edge.start_id,
        end_id=edge.end_id,
        edge_type=edge.edge_type,
        map_name=edge.map_name,
        distance=polyline_length(points),
        waypoints=to_waypoint_list(points),
        floor=None,
        status="ok",
        note="电梯/跨层转移边"
    )


# =========================
# Theta stub
# =========================

def build_theta_placeholder(edge: EdgeDef, start_node: Node, end_node: Node) -> EdgeCacheRecord:
    return EdgeCacheRecord(
        start_id=edge.start_id,
        end_id=edge.end_id,
        edge_type=edge.edge_type,
        map_name=edge.map_name,
        distance=None,
        waypoints=[],
        floor=infer_floor_for_edge(start_node, end_node, edge.edge_type),
        status="pending_theta",
        note="等待 Theta* 规划"
    )


# =========================
# Main builder
# =========================

def build_edge_cache(
    nodes: Dict[str, Node],
    edge_defs: List[EdgeDef],
    fixed_routes: Dict[str, List[List[float]]]
) -> List[EdgeCacheRecord]:
    results: List[EdgeCacheRecord] = []

    for edge in edge_defs:
        if edge.start_id not in nodes:
            results.append(
                EdgeCacheRecord(
                    start_id=edge.start_id,
                    end_id=edge.end_id,
                    edge_type=edge.edge_type,
                    map_name=edge.map_name,
                    distance=None,
                    waypoints=[],
                    floor=None,
                    status="error",
                    note=f"start_id 不存在: {edge.start_id}"
                )
            )
            continue

        if edge.end_id not in nodes:
            results.append(
                EdgeCacheRecord(
                    start_id=edge.start_id,
                    end_id=edge.end_id,
                    edge_type=edge.edge_type,
                    map_name=edge.map_name,
                    distance=None,
                    waypoints=[],
                    floor=None,
                    status="error",
                    note=f"end_id 不存在: {edge.end_id}"
                )
            )
            continue

        start_node = nodes[edge.start_id]
        end_node = nodes[edge.end_id]

        if edge.edge_type == "fixed":
            rec = build_fixed_record(edge, start_node, end_node, fixed_routes)

        elif edge.edge_type == "transfer":
            rec = build_transfer_record(edge, start_node, end_node)

        elif edge.edge_type == "theta":
            rec = build_theta_placeholder(edge, start_node, end_node)

        else:
            rec = EdgeCacheRecord(
                start_id=edge.start_id,
                end_id=edge.end_id,
                edge_type=edge.edge_type,
                map_name=edge.map_name,
                distance=None,
                waypoints=[],
                floor=None,
                status="error",
                note=f"未知 edge_type: {edge.edge_type}"
            )

        results.append(rec)

    return results


def save_edge_cache(edge_cache: List[EdgeCacheRecord], out_json: str) -> None:
    data = [asdict(r) for r in edge_cache]
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main():
    repository_root = Path(__file__).resolve().parents[3]
    nodes_csv = repository_root / "data" / "examples" / "scene_nodes.csv"
    edge_file = repository_root / "data" / "examples" / "edge_definition.csv"
    fixed_routes_json = repository_root / "data" / "examples" / "fixed_routes.json"
    out_json = repository_root / "outputs" / "edge_cache.json"

    nodes = load_nodes(nodes_csv)
    edge_defs = load_edge_defs(edge_file)
    fixed_routes = load_fixed_routes(fixed_routes_json)

    edge_cache = build_edge_cache(nodes, edge_defs, fixed_routes)
    save_edge_cache(edge_cache, out_json)

    print(f"[OK] edge cache saved to: {out_json}")
    print("Summary:")
    for rec in edge_cache:
        print(f"  {rec.start_id} -> {rec.end_id} | {rec.edge_type} | {rec.status}")

if __name__ == "__main__":
    main()
