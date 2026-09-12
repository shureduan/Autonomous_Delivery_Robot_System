import json
import math
import heapq
import random
from pathlib import Path
from collections import defaultdict
import pandas as pd


# =========================================================
# Paths
# =========================================================
BASE_DIR = Path(__file__).resolve().parents[3]

EDGE_CACHE_JSON = BASE_DIR / "data" / "processed" / "edge_cache_merged_all.json"

OUT_MATRIX_CSV = BASE_DIR / "outputs" / "task_node_distance_matrix.csv"
OUT_PAIR_CSV = BASE_DIR / "outputs" / "task_node_pair_shortest_paths.csv"
OUT_XLSX = BASE_DIR / "outputs" / "task_input.xlsx"


# =========================================================
# Task nodes only
# =========================================================
TASK_NODE_MAP = {
    "Ground_Depot": "Depot",
    "Region_A_Floor1": "A",
    "Region_B_Floor1": "B",
    "Region_C_Floor2": "C",
}

TASK_NODE_ORDER = ["Depot", "A", "B", "C"]


# =========================================================
# Helpers
# =========================================================
def load_json(path: Path):
    if not path.exists():
        print(f"[WARN] Missing file: {path}")
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def euclidean(p1, p2):
    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2 +
        (p1[2] - p2[2]) ** 2
    )


def polyline_length(points):
    if not points or len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(len(points) - 1):
        total += euclidean(points[i], points[i + 1])
    return total


def dijkstra(graph, start):
    pq = [(0.0, start)]
    dist = {start: 0.0}
    prev = {}

    while pq:
        cur_dist, u = heapq.heappop(pq)
        if cur_dist > dist[u]:
            continue

        for v, w, edge_name in graph[u]:
            nd = cur_dist + w
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = (u, edge_name)
                heapq.heappush(pq, (nd, v))

    return dist, prev


def reconstruct_path(prev, start, end):
    if start == end:
        return [start]
    if end not in prev:
        return []

    path = [end]
    cur = end
    while cur != start:
        cur = prev[cur][0]
        path.append(cur)
    path.reverse()
    return path


def build_random_demands(task_labels, depot_name="Depot", seed=42):
    """
    生成需求表：
    - Depot: x=y=z=0
    - 其他节点: x,y,z 随机取 0,5,10,...,30
    """
    rng = random.Random(seed)
    choices = list(range(0, 31, 5))

    rows = []
    for node in task_labels:
        if node == depot_name:
            rows.append({
                "node": node,
                "x": 0,
                "y": 0,
                "z": 0,
            })
        else:
            rows.append({
                "node": node,
                "x": rng.choice(choices),
                "y": rng.choice(choices),
                "z": rng.choice(choices),
            })
    return pd.DataFrame(rows)


# =========================================================
# Main
# =========================================================
def main():
    edge_data = load_json(EDGE_CACHE_JSON)
    if edge_data is None:
        print(f"[ERROR] Cannot load: {EDGE_CACHE_JSON}")
        return

    if not isinstance(edge_data, list):
        print("[ERROR] edge cache json is not a list.")
        return

    # -----------------------------------------------------
    # Build graph from all valid edges
    # -----------------------------------------------------
    graph = defaultdict(list)
    edge_rows = []

    for e in edge_data:
        if not isinstance(e, dict):
            continue

        start_id = e.get("start_id")
        end_id = e.get("end_id")
        edge_type = e.get("edge_type", "")
        map_name = e.get("map_name", "")
        distance = e.get("distance")
        waypoints = e.get("waypoints", [])
        floor = e.get("floor")
        status = e.get("status", "")
        note = e.get("note", "")

        if not start_id or not end_id:
            continue

        if status != "ok":
            continue

        if distance is None:
            if waypoints and len(waypoints) >= 2:
                distance = polyline_length(waypoints)
            else:
                continue

        distance = float(distance)

        graph[start_id].append((end_id, distance, f"{start_id}->{end_id}"))
        graph[end_id].append((start_id, distance, f"{start_id}->{end_id}"))

        edge_rows.append({
            "edge_name": f"{start_id}->{end_id}",
            "from": start_id,
            "to": end_id,
            "distance": distance,
            "edge_type": edge_type,
            "map_name": map_name,
            "floor": floor,
            "status": status,
            "note": note,
        })

    edge_df = pd.DataFrame(edge_rows)

    # -----------------------------------------------------
    # Only keep task nodes
    # -----------------------------------------------------
    original_task_nodes = list(TASK_NODE_MAP.keys())

    matrix = pd.DataFrame(index=TASK_NODE_ORDER, columns=TASK_NODE_ORDER, dtype=float)
    pair_rows = []

    for orig_s in original_task_nodes:
        label_s = TASK_NODE_MAP[orig_s]
        dist, prev = dijkstra(graph, orig_s)

        for orig_t in original_task_nodes:
            label_t = TASK_NODE_MAP[orig_t]

            if orig_s == orig_t:
                matrix.loc[label_s, label_t] = 0
                pair_rows.append({
                    "from": label_s,
                    "to": label_t,
                    "shortest_distance": 0,
                    "topology_path": label_s
                })
            elif orig_t in dist:
                path_nodes = reconstruct_path(prev, orig_s, orig_t)
                pretty_path = " -> ".join([TASK_NODE_MAP.get(x, x) for x in path_nodes])

                rounded_dist = int(round(dist[orig_t]))
                matrix.loc[label_s, label_t] = rounded_dist
                pair_rows.append({
                    "from": label_s,
                    "to": label_t,
                    "shortest_distance": rounded_dist,
                    "topology_path": pretty_path
                })
            else:
                matrix.loc[label_s, label_t] = pd.NA
                pair_rows.append({
                    "from": label_s,
                    "to": label_t,
                    "shortest_distance": None,
                    "topology_path": ""
                })

    pair_df = pd.DataFrame(pair_rows)

    # -----------------------------------------------------
    # Build Sheet2 demands
    # -----------------------------------------------------
    demand_df = build_random_demands(TASK_NODE_ORDER, depot_name="Depot", seed=42)

    # -----------------------------------------------------
    # Save csv
    # -----------------------------------------------------
    matrix = matrix.astype("Int64")
    matrix_for_csv = matrix.copy()
    matrix_for_csv.insert(0, "node", matrix_for_csv.index)

    OUT_MATRIX_CSV.parent.mkdir(parents=True, exist_ok=True)

    matrix_for_csv.to_csv(OUT_MATRIX_CSV, index=False)
    pair_df.to_csv(OUT_PAIR_CSV, index=False)

    print(f"[DONE] Saved:")
    print(f"  - {OUT_MATRIX_CSV}")
    print(f"  - {OUT_PAIR_CSV}")

    # -----------------------------------------------------
    # Save xlsx
    # Sheet1: distance matrix
    # Sheet2: demand table
    # -----------------------------------------------------
    try:
        with pd.ExcelWriter(OUT_XLSX, engine="openpyxl") as writer:
            matrix_for_csv.to_excel(writer, sheet_name="Sheet1", index=False)
            demand_df.to_excel(writer, sheet_name="Sheet2", index=False)
            pair_df.to_excel(writer, sheet_name="all_pairs_shortest", index=False)
            edge_df.to_excel(writer, sheet_name="direct_edges", index=False)

        print(f"  - {OUT_XLSX}")

    except ModuleNotFoundError:
        print("[WARN] openpyxl not installed, skipped xlsx export.")


if __name__ == "__main__":
    main()
