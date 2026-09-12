import json
import math
import heapq
from pathlib import Path
from collections import defaultdict
import pandas as pd


# =========================================================
# Fixed paths
# =========================================================
BASE_DIR = Path(__file__).resolve().parents[3]

SOLUTION_XLSX = BASE_DIR / "outputs" / "solution_output.xlsx"
EDGE_CACHE_JSON = BASE_DIR / "data" / "processed" / "edge_cache_merged_all.json"
OUT_JSON = BASE_DIR / "outputs" / "combined_route_plan.json"
OUT_CSV = BASE_DIR / "outputs" / "unity_combined_route.csv"


# =========================================================
# Config
# =========================================================
WAIT_SECONDS_AT_GOAL = 5.0
WAIT_SECONDS_AT_ELEVATOR = 10.0

ELEVATOR_WAIT_NODES = {
    "Platform_Elevator_Point",
    "Floor2_Elevator_Point",
}

GOAL_WAIT_NODES = {
    "Region_A_Floor1",
    "Region_B_Floor1",
    "Region_C_Floor2",
    # 如果以后你想给卸货区也加停留，可以打开下面这行
    # "Ground_Depot",
}

# task-node name mapping: solution_output uses short names,
# edge cache uses full topology node names
TASK_TO_TOPOLOGY = {
    "Depot": "Ground_Depot",
    "A": "Region_A_Floor1",
    "B": "Region_B_Floor1",
    "C": "Region_C_Floor2",
}

# reverse map for convenience
TOPOLOGY_TO_TASK = {v: k for k, v in TASK_TO_TOPOLOGY.items()}


# =========================================================
# Helpers
# =========================================================
def load_json(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def euclidean(p1, p2):
    return math.sqrt(
        (float(p1[0]) - float(p2[0])) ** 2
        + (float(p1[1]) - float(p2[1])) ** 2
        + (float(p1[2]) - float(p2[2])) ** 2
    )


def points_equal(p1, p2, eps=1e-6):
    return euclidean(p1, p2) <= eps


def normalize_waypoints(wps):
    out = []
    for p in wps:
        out.append([float(p[0]), float(p[1]), float(p[2])])
    return out


def node_wait_seconds(node_id: str) -> float:
    if node_id in ELEVATOR_WAIT_NODES:
        return WAIT_SECONDS_AT_ELEVATOR
    if node_id in GOAL_WAIT_NODES:
        return WAIT_SECONDS_AT_GOAL
    return 0.0


def node_wait_tag(node_id: str) -> str:
    if node_id in ELEVATOR_WAIT_NODES:
        return f"wait_at_{node_id}"
    if node_id in GOAL_WAIT_NODES:
        return f"wait_at_{node_id}"
    return "move"


# =========================================================
# Read trip sheet
# =========================================================
def read_solution_trips(solution_xlsx: Path):
    xls = pd.ExcelFile(solution_xlsx)

    # 优先找 trip_sequence，否则退回第二个sheet
    if "trip_sequence" in xls.sheet_names:
        df = pd.read_excel(solution_xlsx, sheet_name="trip_sequence")
    else:
        if len(xls.sheet_names) < 2:
            raise ValueError("solution_output.xlsx must contain at least 2 sheets.")
        df = pd.read_excel(solution_xlsx, sheet_name=xls.sheet_names[1])

    if "route" not in df.columns:
        raise ValueError("trip sheet must contain a 'route' column.")

    trips = []
    for idx, row in df.iterrows():
        route_str = str(row["route"]).strip()
        if not route_str:
            continue

        route_nodes = [x.strip() for x in route_str.split("->") if str(x).strip()]
        trip_id = int(row["trip_id"]) if "trip_id" in df.columns and not pd.isna(row["trip_id"]) else idx + 1

        trips.append({
            "trip_id": trip_id,
            "route_task_nodes": route_nodes,
            "distance": None if "distance" not in df.columns or pd.isna(row["distance"]) else float(row["distance"]),
            "load_total": None if "load_total" not in df.columns or pd.isna(row["load_total"]) else float(row["load_total"]),
        })

    return trips


# =========================================================
# Edge cache graph
# =========================================================
def build_graph(edge_data):
    """
    Build an undirected graph from edge cache.
    Keep direct edge waypoint geometry in both directions.
    """
    graph = defaultdict(list)
    edge_waypoints = {}

    for e in edge_data:
        if not isinstance(e, dict):
            continue

        if e.get("status") != "ok":
            continue

        u = e.get("start_id")
        v = e.get("end_id")
        d = e.get("distance")
        wps = e.get("waypoints", [])

        if not u or not v:
            continue

        if d is None:
            if len(wps) >= 2:
                d = 0.0
                for i in range(len(wps) - 1):
                    d += euclidean(wps[i], wps[i + 1])
            else:
                continue

        d = float(d)
        wps = normalize_waypoints(wps)

        graph[u].append((v, d))
        graph[v].append((u, d))

        edge_waypoints[(u, v)] = wps
        edge_waypoints[(v, u)] = list(reversed(wps))

    return graph, edge_waypoints


def dijkstra(graph, start):
    pq = [(0.0, start)]
    dist = {start: 0.0}
    prev = {}

    while pq:
        cur_dist, u = heapq.heappop(pq)
        if cur_dist > dist[u]:
            continue

        for v, w in graph[u]:
            nd = cur_dist + w
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    return dist, prev


def reconstruct_node_path(prev, start, end):
    if start == end:
        return [start]
    if end not in prev:
        return []

    path = [end]
    cur = end
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


# =========================================================
# Stitch topology path into continuous waypoints
# =========================================================
def topology_path_to_waypoints(path_nodes, edge_waypoints):
    """
    path_nodes like:
      Ground_Depot -> Ground_Ramp2_Bottom -> Platform_Ramp2_Top -> Region_B_Floor1

    returns:
      continuous waypoint list
    """
    if not path_nodes:
        return []

    if len(path_nodes) == 1:
        return []

    merged = []

    for i in range(len(path_nodes) - 1):
        u = path_nodes[i]
        v = path_nodes[i + 1]

        if (u, v) not in edge_waypoints:
            raise KeyError(f"Missing direct edge waypoints for {u} -> {v}")

        seg = edge_waypoints[(u, v)]
        if not seg:
            continue

        if not merged:
            merged.extend(seg)
        else:
            if points_equal(merged[-1], seg[0]):
                merged.extend(seg[1:])
            else:
                merged.extend(seg)

    return merged


def append_wait_entry(flattened_entries, seq_counter, trip_id, node_id):
    if not flattened_entries:
        return seq_counter

    wait_sec = node_wait_seconds(node_id)
    if wait_sec <= 0.0:
        return seq_counter

    p = flattened_entries[-1]

    # 避免连续重复插入同一个 wait
    if len(flattened_entries) > 0:
        last = flattened_entries[-1]
        if (
            last["tag"] == node_wait_tag(node_id)
            and abs(float(last["wait_sec"]) - wait_sec) < 1e-9
            and points_equal([last["x"], last["y"], last["z"]], [p["x"], p["y"], p["z"]])
        ):
            return seq_counter

    wait_entry = {
        "seq": seq_counter,
        "trip_id": trip_id,
        "x": p["x"],
        "y": p["y"],
        "z": p["z"],
        "wait_sec": wait_sec,
        "tag": node_wait_tag(node_id),
    }
    flattened_entries.append(wait_entry)
    return seq_counter + 1


def build_trip_waypoint_entries(trip_id, task_route, graph, edge_waypoints):
    """
    task_route: ['Depot', 'B', 'Depot']
    convert task nodes -> topology shortest paths -> continuous waypoint entries

    Each entry:
      {
        "seq": ...,
        "trip_id": ...,
        "x": ...,
        "y": ...,
        "z": ...,
        "wait_sec": ...,
        "tag": ...
      }
    """
    if len(task_route) < 2:
        return [], []

    topology_route_segments = []
    flattened_entries = []
    seq_counter = 0

    for pair_idx in range(len(task_route) - 1):
        task_u = task_route[pair_idx]
        task_v = task_route[pair_idx + 1]

        topo_u = TASK_TO_TOPOLOGY.get(task_u, task_u)
        topo_v = TASK_TO_TOPOLOGY.get(task_v, task_v)

        dist, prev = dijkstra(graph, topo_u)
        if topo_v not in dist:
            raise RuntimeError(f"No topology path found from {topo_u} to {topo_v}")

        topo_node_path = reconstruct_node_path(prev, topo_u, topo_v)

        # 仅用于摘要展示
        seg_waypoints_all = topology_path_to_waypoints(topo_node_path, edge_waypoints)

        topology_route_segments.append({
            "from_task": task_u,
            "to_task": task_v,
            "from_topology": topo_u,
            "to_topology": topo_v,
            "topology_node_path": topo_node_path,
            "num_waypoints": len(seg_waypoints_all),
            "shortest_distance": dist[topo_v],
        })

        # 逐条 direct edge 拼接，这样可以在每个 edge 的 end_id 处精确插 wait
        for edge_idx in range(len(topo_node_path) - 1):
            u = topo_node_path[edge_idx]
            v = topo_node_path[edge_idx + 1]

            if (u, v) not in edge_waypoints:
                raise KeyError(f"Missing direct edge waypoints for {u} -> {v}")

            seg = edge_waypoints[(u, v)]
            if not seg:
                continue

            for p in seg:
                if flattened_entries:
                    last = flattened_entries[-1]
                    if points_equal([last["x"], last["y"], last["z"]], p):
                        continue

                entry = {
                    "seq": seq_counter,
                    "trip_id": trip_id,
                    "x": float(p[0]),
                    "y": float(p[1]),
                    "z": float(p[2]),
                    "wait_sec": 0.0,
                    "tag": "move",
                }
                flattened_entries.append(entry)
                seq_counter += 1

            # 到达这条 edge 的终点节点 v 后，按节点类型插 wait
            seq_counter = append_wait_entry(
                flattened_entries=flattened_entries,
                seq_counter=seq_counter,
                trip_id=trip_id,
                node_id=v,
            )

    return topology_route_segments, flattened_entries


# =========================================================
# Main
# =========================================================
def main():
    edge_data = load_json(EDGE_CACHE_JSON)
    trips = read_solution_trips(SOLUTION_XLSX)

    graph, edge_waypoints = build_graph(edge_data)

    all_trip_results = []
    flat_rows = []

    global_seq = 0

    for trip in trips:
        trip_id = trip["trip_id"]
        task_route = trip["route_task_nodes"]

        topo_segments, wp_entries = build_trip_waypoint_entries(
            trip_id=trip_id,
            task_route=task_route,
            graph=graph,
            edge_waypoints=edge_waypoints,
        )

        # add global seq
        for row in wp_entries:
            row["global_seq"] = global_seq
            global_seq += 1
            flat_rows.append(row)

        all_trip_results.append({
            "trip_id": trip_id,
            "route_task_nodes": task_route,
            "distance_from_solver": trip.get("distance"),
            "load_total": trip.get("load_total"),
            "topology_segments": topo_segments,
            "waypoints": wp_entries,
        })

    out_obj = {
        "source_solution_xlsx": str(SOLUTION_XLSX),
        "source_edge_cache_json": str(EDGE_CACHE_JSON),
        "wait_seconds_at_goal": WAIT_SECONDS_AT_GOAL,
        "wait_seconds_at_elevator": WAIT_SECONDS_AT_ELEVATOR,
        "goal_wait_nodes": sorted(list(GOAL_WAIT_NODES)),
        "elevator_wait_nodes": sorted(list(ELEVATOR_WAIT_NODES)),
        "trips": all_trip_results,
    }

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(out_obj, f, ensure_ascii=False, indent=2)

    df = pd.DataFrame(flat_rows)

    # 列顺序固定一下，方便 UnityRouteExecutor 读取
    preferred_cols = ["global_seq", "seq", "trip_id", "x", "y", "z", "wait_sec", "tag"]
    existing_cols = [c for c in preferred_cols if c in df.columns]
    other_cols = [c for c in df.columns if c not in existing_cols]
    df = df[existing_cols + other_cols]

    df.to_csv(OUT_CSV, index=False)

    print("[DONE] Saved:")
    print(f"  JSON: {OUT_JSON}")
    print(f"  CSV : {OUT_CSV}")

    print("\nTrip summary:")
    for t in all_trip_results:
        print(
            f"  Trip {t['trip_id']}: "
            f"{' -> '.join(t['route_task_nodes'])} | "
            f"{len(t['waypoints'])} waypoint entries"
        )


if __name__ == "__main__":
    main()
