from __future__ import annotations

import csv
import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
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
    def pos3(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass
class PlanningMap:
    occ_grid: np.ndarray          # True = obstacle, False = free
    prob_map: np.ndarray
    explored_mask: np.ndarray     # bool
    res: float
    origin_x: float
    origin_z: float
    height: int
    width: int


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


def inflate_obstacles(binary_occ: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return binary_occ.copy()

    h, w = binary_occ.shape
    inflated = binary_occ.copy()
    occ_indices = np.argwhere(binary_occ)

    for r, c in occ_indices:
        r0 = max(0, r - radius_cells)
        r1 = min(h, r + radius_cells + 1)
        c0 = max(0, c - radius_cells)
        c1 = min(w, c + radius_cells + 1)

        for rr in range(r0, r1):
            for cc in range(c0, c1):
                if (rr - r) ** 2 + (cc - c) ** 2 <= radius_cells ** 2:
                    inflated[rr, cc] = True

    return inflated


def load_planning_map(
    npz_path: str,
    occ_thresh: float,
    treat_unknown_as_obstacle: bool,
    inflation_radius_m: float,
    valid_world_rect: Optional[Tuple[float, float, float, float]] = None,
) -> PlanningMap:
    data = np.load(npz_path)

    prob_map = data["prob_map"].astype(np.float32)
    explored_mask = data["explored_mask"].astype(bool)

    res = float(data["map_res"])
    origin_x = float(data["map_x_min"])
    origin_z = float(data["map_z_min"])

    occ_grid = prob_map >= occ_thresh
    if treat_unknown_as_obstacle:
        occ_grid = occ_grid | (~explored_mask)

    inflation_radius_cells = int(math.ceil(inflation_radius_m / res))
    print(f"[MAP] npz={npz_path}")
    print(f"[MAP] occ_thresh={occ_thresh}, unknown_as_obstacle={treat_unknown_as_obstacle}")
    print(f"[MAP] inflation_radius_m={inflation_radius_m}, cells={inflation_radius_cells}")

    occ_grid = inflate_obstacles(occ_grid, inflation_radius_cells)

    h, w = occ_grid.shape

    if valid_world_rect is not None:
        min_x, max_x, min_z, max_z = valid_world_rect
        print(f"[MAP] valid_world_rect = {valid_world_rect}")

        for row in range(h):
            for col in range(w):
                x = origin_x + (col + 0.5) * res
                z = origin_z + (row + 0.5) * res
                if not (min_x <= x <= max_x and min_z <= z <= max_z):
                    occ_grid[row, col] = True

    return PlanningMap(
        occ_grid=occ_grid,
        prob_map=prob_map,
        explored_mask=explored_mask,
        res=res,
        origin_x=origin_x,
        origin_z=origin_z,
        height=h,
        width=w,
    )


# =========================
# Coordinate conversion
# =========================

def world_to_grid(x: float, z: float, pm: PlanningMap) -> Tuple[int, int]:
    col = int(math.floor((x - pm.origin_x) / pm.res))
    row = int(math.floor((z - pm.origin_z) / pm.res))
    return row, col


def grid_to_world(row: int, col: int, y: float, pm: PlanningMap) -> Tuple[float, float, float]:
    x = pm.origin_x + (col + 0.5) * pm.res
    z = pm.origin_z + (row + 0.5) * pm.res
    return (x, y, z)


def in_bounds(row: int, col: int, pm: PlanningMap) -> bool:
    return 0 <= row < pm.height and 0 <= col < pm.width


def is_free(row: int, col: int, pm: PlanningMap) -> bool:
    return in_bounds(row, col, pm) and (not pm.occ_grid[row, col])


def find_nearest_free_cell(
    start_rc: Tuple[int, int],
    pm: PlanningMap,
    max_radius: int
) -> Optional[Tuple[int, int]]:
    sr, sc = start_rc
    if is_free(sr, sc, pm):
        return (sr, sc)

    for rad in range(1, max_radius + 1):
        r0 = max(0, sr - rad)
        r1 = min(pm.height, sr + rad + 1)
        c0 = max(0, sc - rad)
        c1 = min(pm.width, sc + rad + 1)

        for r in range(r0, r1):
            for c in range(c0, c1):
                if max(abs(r - sr), abs(c - sc)) != rad:
                    continue
                if is_free(r, c, pm):
                    return (r, c)
    return None


# =========================
# Theta* helpers
# =========================

def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    (r1, c1), (r2, c2) = a, b
    return math.hypot(r2 - r1, c2 - c1)


def dist(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    (r1, c1), (r2, c2) = a, b
    return math.hypot(r2 - r1, c2 - c1)


def get_neighbors(rc: Tuple[int, int], pm: PlanningMap) -> List[Tuple[int, int]]:
    r, c = rc
    nbrs: List[Tuple[int, int]] = []

    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    for dr, dc in directions:
        nr, nc = r + dr, c + dc
        if not is_free(nr, nc, pm):
            continue

        if dr != 0 and dc != 0:
            if not is_free(r + dr, c, pm) or not is_free(r, c + dc, pm):
                continue

        nbrs.append((nr, nc))

    return nbrs


def line_of_sight(a: Tuple[int, int], b: Tuple[int, int], pm: PlanningMap) -> bool:
    r0, c0 = a
    r1, c1 = b

    dr = r1 - r0
    dc = c1 - c0
    steps = int(max(abs(dr), abs(dc)))

    if steps == 0:
        return is_free(r0, c0, pm)

    for i in range(steps + 1):
        t = i / steps
        rr = int(round(r0 + dr * t))
        cc = int(round(c0 + dc * t))
        if not is_free(rr, cc, pm):
            return False

    return True


def reconstruct_path(
    parents: Dict[Tuple[int, int], Tuple[int, int]],
    goal: Tuple[int, int]
) -> List[Tuple[int, int]]:
    path = [goal]
    cur = goal
    while parents[cur] != cur:
        cur = parents[cur]
        path.append(cur)
    path.reverse()
    return path


# =========================
# Basic Theta*
# =========================

def theta_star(
    pm: PlanningMap,
    start: Tuple[int, int],
    goal: Tuple[int, int]
) -> Optional[List[Tuple[int, int]]]:
    if not is_free(start[0], start[1], pm):
        return None
    if not is_free(goal[0], goal[1], pm):
        return None

    open_heap: List[Tuple[float, int, Tuple[int, int]]] = []
    g: Dict[Tuple[int, int], float] = {}
    parents: Dict[Tuple[int, int], Tuple[int, int]] = {}
    closed: set[Tuple[int, int]] = set()

    counter = 0
    g[start] = 0.0
    parents[start] = start
    heapq.heappush(open_heap, (heuristic(start, goal), counter, start))

    while open_heap:
        _, _, s = heapq.heappop(open_heap)

        if s in closed:
            continue

        if s == goal:
            return reconstruct_path(parents, goal)

        closed.add(s)

        for sp in get_neighbors(s, pm):
            if sp in closed:
                continue

            if sp not in g:
                g[sp] = math.inf
                parents[sp] = None  # type: ignore

            ps = parents[s]
            if ps is not None and line_of_sight(ps, sp, pm):
                tentative_g = g[ps] + dist(ps, sp)
                tentative_parent = ps
            else:
                tentative_g = g[s] + dist(s, sp)
                tentative_parent = s

            if tentative_g < g[sp]:
                g[sp] = tentative_g
                parents[sp] = tentative_parent
                counter += 1
                f = tentative_g + heuristic(sp, goal)
                heapq.heappush(open_heap, (f, counter, sp))

    return None


# =========================
# Path utilities
# =========================

def grid_path_to_world_waypoints(
    path_rc: List[Tuple[int, int]],
    y_value: float,
    pm: PlanningMap
) -> List[List[float]]:
    points = [grid_to_world(r, c, y_value, pm) for (r, c) in path_rc]
    return [[float(x), float(y), float(z)] for x, y, z in points]


def world_polyline_length(points: List[List[float]]) -> float:
    if len(points) < 2:
        return 0.0

    total = 0.0
    for i in range(len(points) - 1):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[i + 1]
        total += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return total


def safe_name(s: str) -> str:
    return s.replace(" ", "_").replace("->", "_to_").replace("/", "_")


# =========================
# Visualization
# =========================

def save_path_visualization(
    pm: PlanningMap,
    path_rc: List[Tuple[int, int]],
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    start_world: Tuple[float, float, float],
    goal_world: Tuple[float, float, float],
    out_png: str,
    title: str = "Theta* Path",
    valid_world_rect: Optional[Tuple[float, float, float, float]] = None,
) -> None:
    vis = np.full(pm.prob_map.shape, 0.5, dtype=np.float32)
    vis[pm.explored_mask] = 1.0 - pm.prob_map[pm.explored_mask]

    x_max = pm.origin_x + pm.width * pm.res
    z_max = pm.origin_z + pm.height * pm.res

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(
        vis,
        origin="lower",
        cmap="gray",
        vmin=0.0,
        vmax=1.0,
        extent=[pm.origin_x, x_max, pm.origin_z, z_max],
    )

    occ_overlay = np.zeros((pm.height, pm.width, 4), dtype=np.float32)
    occ_overlay[pm.occ_grid] = [1.0, 0.0, 0.0, 0.18]
    ax.imshow(
        occ_overlay,
        origin="lower",
        extent=[pm.origin_x, x_max, pm.origin_z, z_max],
    )

    if path_rc:
        y_value = start_world[1]
        path_world = [grid_to_world(r, c, y_value, pm) for (r, c) in path_rc]
        path_x = [p[0] for p in path_world]
        path_z = [p[2] for p in path_world]
        ax.plot(path_x, path_z, "b-", linewidth=2, label="Theta* path")
        ax.scatter(path_x, path_z, s=18, c="blue")

    ax.scatter([start_world[0]], [start_world[2]], s=80, c="lime", marker="o", label="start")
    ax.scatter([goal_world[0]], [goal_world[2]], s=80, c="red", marker="x", label="goal")

    start_snap_world = grid_to_world(start_rc[0], start_rc[1], start_world[1], pm)
    goal_snap_world = grid_to_world(goal_rc[0], goal_rc[1], goal_world[1], pm)
    ax.scatter([start_snap_world[0]], [start_snap_world[2]], s=50, c="green", marker="s", label="start snapped")
    ax.scatter([goal_snap_world[0]], [goal_snap_world[2]], s=50, c="darkred", marker="s", label="goal snapped")

    ax.text(start_world[0] + 0.3, start_world[2] + 0.3, "start", color="lime")
    ax.text(goal_world[0] + 0.3, goal_world[2] + 0.3, "goal", color="red")

    if valid_world_rect is not None:
        min_x, max_x, min_z, max_z = valid_world_rect
        rect_x = [min_x, max_x, max_x, min_x, min_x]
        rect_z = [min_z, min_z, max_z, max_z, min_z]
        ax.plot(rect_x, rect_z, "y--", linewidth=2, label="valid boundary")

    ax.set_title(title)
    ax.set_xlabel("world x")
    ax.set_ylabel("world z")
    ax.set_aspect("equal", "box")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


def save_layer_overview(
    pm: PlanningMap,
    records: List[dict],
    out_png: str,
    title: str,
    valid_world_rect: Optional[Tuple[float, float, float, float]] = None,
) -> None:
    vis = np.full(pm.prob_map.shape, 0.5, dtype=np.float32)
    vis[pm.explored_mask] = 1.0 - pm.prob_map[pm.explored_mask]

    x_max = pm.origin_x + pm.width * pm.res
    z_max = pm.origin_z + pm.height * pm.res

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(
        vis,
        origin="lower",
        cmap="gray",
        vmin=0.0,
        vmax=1.0,
        extent=[pm.origin_x, x_max, pm.origin_z, z_max],
    )

    occ_overlay = np.zeros((pm.height, pm.width, 4), dtype=np.float32)
    occ_overlay[pm.occ_grid] = [1.0, 0.0, 0.0, 0.14]
    ax.imshow(
        occ_overlay,
        origin="lower",
        extent=[pm.origin_x, x_max, pm.origin_z, z_max],
    )

    for rec in records:
        if rec["status"] != "ok":
            continue
        waypoints = rec.get("waypoints_data", [])
        if not waypoints:
            continue

        xs = [p[0] for p in waypoints]
        zs = [p[2] for p in waypoints]
        ax.plot(xs, zs, linewidth=2, label=f'{rec["start_id"]} -> {rec["end_id"]}')
        ax.scatter([xs[0]], [zs[0]], s=30, marker="o")
        ax.scatter([xs[-1]], [zs[-1]], s=30, marker="x")

    if valid_world_rect is not None:
        min_x, max_x, min_z, max_z = valid_world_rect
        rect_x = [min_x, max_x, max_x, min_x, min_x]
        rect_z = [min_z, min_z, max_z, max_z, min_z]
        ax.plot(rect_x, rect_z, "y--", linewidth=2, label="valid boundary")

    ax.set_title(title)
    ax.set_xlabel("world x")
    ax.set_ylabel("world z")
    ax.set_aspect("equal", "box")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=220)
    plt.close(fig)


# =========================
# CSV output
# =========================

def save_waypoints_csv(
    waypoints: List[List[float]],
    out_csv: str,
    start_id: str,
    end_id: str,
) -> None:
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y", "z", "start_id", "end_id"])
        for i, p in enumerate(waypoints):
            writer.writerow([i, p[0], p[1], p[2], start_id, end_id])


def save_summary_csv(
    records: List[dict],
    out_csv: str
) -> None:
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "layer",
            "start_id", "end_id", "status",
            "distance", "num_waypoints",
            "png_path", "waypoint_csv_path"
        ])
        for r in records:
            writer.writerow([
                r["layer"],
                r["start_id"],
                r["end_id"],
                r["status"],
                r["distance"],
                r["num_waypoints"],
                r["png_path"],
                r["waypoint_csv_path"],
            ])


# =========================
# Single-edge test
# =========================

def test_single_edge(
    nodes: Dict[str, Node],
    pm: PlanningMap,
    start_id: str,
    end_id: str,
    layer_name: str,
    out_png: Optional[str] = None,
    out_waypoint_csv: Optional[str] = None,
    valid_world_rect: Optional[Tuple[float, float, float, float]] = None,
) -> dict:
    if start_id not in nodes:
        raise ValueError(f"找不到 start_id: {start_id}")
    if end_id not in nodes:
        raise ValueError(f"找不到 end_id: {end_id}")

    start_node = nodes[start_id]
    end_node = nodes[end_id]

    start_rc_raw = world_to_grid(start_node.x, start_node.z, pm)
    end_rc_raw = world_to_grid(end_node.x, end_node.z, pm)

    start_rc = find_nearest_free_cell(start_rc_raw, pm, max_radius=CONFIG["snap_max_radius"])
    end_rc = find_nearest_free_cell(end_rc_raw, pm, max_radius=CONFIG["snap_max_radius"])

    print("=" * 80)
    print(f"[EDGE] {start_id} -> {end_id}")
    print(f"start world = {start_node.pos3}")
    print(f"end world   = {end_node.pos3}")
    print(f"start grid raw = {start_rc_raw}, snapped = {start_rc}")
    print(f"end grid raw   = {end_rc_raw}, snapped = {end_rc}")

    result = {
        "layer": layer_name,
        "start_id": start_id,
        "end_id": end_id,
        "status": "error",
        "distance": "",
        "num_waypoints": 0,
        "png_path": out_png or "",
        "waypoint_csv_path": out_waypoint_csv or "",
        "waypoints_data": [],
    }

    if start_rc is None:
        print("[ERROR] 起点附近找不到可通行栅格")
        result["status"] = "invalid_start"
        return result

    if end_rc is None:
        print("[ERROR] 终点附近找不到可通行栅格")
        result["status"] = "invalid_goal"
        return result

    path_rc = theta_star(pm, start_rc, end_rc)
    if path_rc is None:
        print("[ERROR] Theta* 未找到路径")
        result["status"] = "no_path"
        return result

    y_value = start_node.y
    waypoints = grid_path_to_world_waypoints(path_rc, y_value, pm)
    distance = world_polyline_length(waypoints)

    print(f"[OK] path found, #waypoints = {len(waypoints)}, distance = {distance:.3f}")
    print("First 10 waypoints:")
    for p in waypoints[:10]:
        print("  ", p)

    if out_png is not None:
        save_path_visualization(
            pm=pm,
            path_rc=path_rc,
            start_rc=start_rc,
            goal_rc=end_rc,
            start_world=start_node.pos3,
            goal_world=end_node.pos3,
            out_png=out_png,
            title=f"{start_id} -> {end_id}",
            valid_world_rect=valid_world_rect,
        )
        print(f"[OK] visualization saved to: {out_png}")

    if out_waypoint_csv is not None:
        save_waypoints_csv(
            waypoints=waypoints,
            out_csv=out_waypoint_csv,
            start_id=start_id,
            end_id=end_id,
        )
        print(f"[OK] waypoint csv saved to: {out_waypoint_csv}")

    result["status"] = "ok"
    result["distance"] = f"{distance:.3f}"
    result["num_waypoints"] = len(waypoints)
    result["waypoints_data"] = waypoints
    return result


# =========================
# Batch runners
# =========================

def run_batch(
    layer_name: str,
    nodes_csv: str,
    npz_map_path: str,
    out_dir: str,
    summary_csv_name: str,
    edges: List[Tuple[str, str]],
    valid_world_rect: Optional[Tuple[float, float, float, float]],
) -> List[dict]:
    out_dir_path = Path(out_dir)
    out_dir_path.mkdir(parents=True, exist_ok=True)

    nodes = load_nodes(nodes_csv)
    pm = load_planning_map(
        npz_path=npz_map_path,
        occ_thresh=CONFIG["occ_thresh"],
        treat_unknown_as_obstacle=CONFIG["treat_unknown_as_obstacle"],
        inflation_radius_m=CONFIG["inflation_radius_m"],
        valid_world_rect=valid_world_rect,
    )

    summary_records: List[dict] = []

    for start_id, end_id in edges:
        edge_name = f"{safe_name(start_id)}_to_{safe_name(end_id)}"
        out_png = str(out_dir_path / f"{edge_name}.png")
        out_csv = str(out_dir_path / f"{edge_name}_waypoints.csv")

        rec = test_single_edge(
            nodes=nodes,
            pm=pm,
            start_id=start_id,
            end_id=end_id,
            layer_name=layer_name,
            out_png=out_png,
            out_waypoint_csv=out_csv,
            valid_world_rect=valid_world_rect,
        )
        summary_records.append(rec)

    summary_csv = str(out_dir_path / summary_csv_name)
    save_summary_csv(summary_records, summary_csv)
    print("=" * 80)
    print(f"[OK] {layer_name} batch summary saved to: {summary_csv}")

    # 每层一张总图
    overview_png = str(out_dir_path / f"{layer_name}_all_routes.png")
    save_layer_overview(
        pm=pm,
        records=summary_records,
        out_png=overview_png,
        title=f"{layer_name} all routes",
        valid_world_rect=valid_world_rect,
    )
    print(f"[OK] {layer_name} overview saved to: {overview_png}")

    return summary_records


def save_global_summary(all_records: List[dict], out_csv: str) -> None:
    save_summary_csv(all_records, out_csv)
    print(f"[OK] global theta summary saved to: {out_csv}")


# =========================
# Config（集中改这里）
# =========================

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]

CONFIG = {
    "base_dir": str(REPOSITORY_ROOT),
    "nodes_csv": "scene_nodes.csv",

    # 平台层
    "platform_npz": "slam_map_platform.npz",
    "platform_output_dir": str(REPOSITORY_ROOT / "outputs" / "theta_star" / "platform"),
    "platform_summary_csv_name": "platform_theta_summary.csv",
    "platform_edges": [
        ("Platform_Ramp2_Top", "Region_A_Floor1"),
        ("Platform_Ramp2_Top", "Region_B_Floor1"),
        ("Platform_Elevator_Point", "Region_A_Floor1"),
        ("Platform_Elevator_Point", "Region_B_Floor1"),
        ("Region_A_Floor1", "Region_B_Floor1"),
    ],

    # 二楼
    "floor2_npz": "slam_map_floor2.npz",
    "floor2_output_dir": str(REPOSITORY_ROOT / "outputs" / "theta_star" / "floor2"),
    "floor2_summary_csv_name": "floor2_theta_summary.csv",
    "floor2_edges": [
        ("Floor2_Elevator_Point", "Region_C_Floor2"),
    ],
    "floor2_valid_world_rect": (-5.0, 15.0, 17.0, 50.0),

    # 总表
    "global_output_dir": str(REPOSITORY_ROOT / "outputs" / "theta_star"),
    "global_summary_csv_name": "theta_summary_all.csv",

    # 公共参数
    "occ_thresh": 0.65,
    "treat_unknown_as_obstacle": True,
    "inflation_radius_m": 0.8,
    "snap_max_radius": 40,
}


# =========================
# Main
# =========================

def main():
    base = Path(CONFIG["base_dir"])
    nodes_csv = str(base / CONFIG["nodes_csv"])

    platform_records = run_batch(
        layer_name="platform",
        nodes_csv=nodes_csv,
        npz_map_path=str(base / CONFIG["platform_npz"]),
        out_dir=CONFIG["platform_output_dir"],
        summary_csv_name=CONFIG["platform_summary_csv_name"],
        edges=CONFIG["platform_edges"],
        valid_world_rect=None,
    )

    floor2_records = run_batch(
        layer_name="floor2",
        nodes_csv=nodes_csv,
        npz_map_path=str(base / CONFIG["floor2_npz"]),
        out_dir=CONFIG["floor2_output_dir"],
        summary_csv_name=CONFIG["floor2_summary_csv_name"],
        edges=CONFIG["floor2_edges"],
        valid_world_rect=CONFIG["floor2_valid_world_rect"],
    )

    all_records = platform_records + floor2_records
    global_summary_path = str(Path(CONFIG["global_output_dir"]) / CONFIG["global_summary_csv_name"])
    save_global_summary(all_records, global_summary_path)


if __name__ == "__main__":
    main()
