import argparse
import json
import math
from pathlib import Path
import pandas as pd


# =========================================================
# Helpers
# =========================================================
def load_json(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Missing json: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(data, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def euclidean(p1, p2):
    return math.sqrt(
        (float(p1[0]) - float(p2[0])) ** 2
        + (float(p1[1]) - float(p2[1])) ** 2
        + (float(p1[2]) - float(p2[2])) ** 2
    )


def polyline_length(points):
    if not points or len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(len(points) - 1):
        total += euclidean(points[i], points[i + 1])
    return total


def normalize_waypoints(points):
    out = []
    for p in points:
        out.append([float(p[0]), float(p[1]), float(p[2])])
    return out


def read_waypoint_csv(csv_path: Path):
    if not csv_path.exists():
        raise FileNotFoundError(f"Waypoint csv not found: {csv_path}")

    df = pd.read_csv(csv_path)
    if df.empty:
        return []

    # 兼容一些常见列名
    lower_cols = {str(c).strip().lower(): c for c in df.columns}

    def pick(*names):
        for n in names:
            if n in lower_cols:
                return lower_cols[n]
        raise KeyError(f"Cannot find any of columns {names} in {csv_path}")

    x_col = pick("x", "world_x", "x_m", "px")
    y_col = pick("y", "world_y", "y_m", "py")
    z_col = pick("z", "world_z", "z_m", "pz")

    waypoints = []
    for _, row in df.iterrows():
        x = row[x_col]
        y = row[y_col]
        z = row[z_col]
        if pd.isna(x) or pd.isna(y) or pd.isna(z):
            continue
        waypoints.append([float(x), float(y), float(z)])

    return waypoints


def summary_to_edges(summary_csv_path: Path, default_map_name: str, default_floor=None):
    if not summary_csv_path.exists():
        print(f"[WARN] Missing summary csv: {summary_csv_path}")
        return []

    df = pd.read_csv(summary_csv_path)
    if df.empty:
        print(f"[WARN] Empty summary csv: {summary_csv_path}")
        return []

    required_cols = [
        "start_id",
        "end_id",
        "status",
        "distance",
        "waypoint_csv_path",
    ]
    for c in required_cols:
        if c not in df.columns:
            raise KeyError(f"{summary_csv_path.name} missing required column: {c}")

    edges = []

    for _, row in df.iterrows():
        start_id = str(row["start_id"]).strip()
        end_id = str(row["end_id"]).strip()
        status = str(row["status"]).strip()

        if not start_id or not end_id:
            continue

        map_name = default_map_name
        if "layer" in df.columns and not pd.isna(row["layer"]):
            map_name = str(row["layer"]).strip()

        distance = None
        if not pd.isna(row["distance"]):
            distance = float(row["distance"])

        waypoint_csv_path = None
        if not pd.isna(row["waypoint_csv_path"]):
            waypoint_csv_path = Path(str(row["waypoint_csv_path"]).strip())

        waypoints = []
        note = f"{default_map_name} theta imported from summary csv"

        if waypoint_csv_path is not None:
            try:
                waypoints = read_waypoint_csv(waypoint_csv_path)
            except Exception as e:
                print(f"[WARN] Failed reading waypoint csv for {start_id}->{end_id}: {e}")
                note += " | waypoint csv unreadable"

        if distance is None and waypoints:
            distance = polyline_length(waypoints)

        if not waypoints:
            # 没有 waypoint 时，不强行标 ok
            if status == "ok":
                status = "pending_theta"
            note += " | waypoint csv missing or empty"

        floor = default_floor
        if "layer" in df.columns and not pd.isna(row["layer"]):
            layer_str = str(row["layer"]).strip().lower()
            if layer_str == "platform":
                floor = 1
            elif layer_str == "floor2":
                floor = 2

        edges.append({
            "start_id": start_id,
            "end_id": end_id,
            "edge_type": "theta",
            "map_name": map_name,
            "distance": distance,
            "waypoints": normalize_waypoints(waypoints),
            "floor": floor,
            "status": status,
            "note": note,
        })

    print(f"[INFO] Parsed {len(edges)} edges from {summary_csv_path.name}")
    return edges


def merge_edges(base_edges, new_edges):
    """
    对同一个 (start_id, end_id) 用新边替换旧边。
    ground 固定边不会被动，因为 summary 不会覆盖它们。
    """
    merged = list(base_edges)
    key_to_idx = {}

    for i, e in enumerate(merged):
        key = (e.get("start_id"), e.get("end_id"))
        key_to_idx[key] = i

    replaced = 0
    added = 0

    for e in new_edges:
        key = (e.get("start_id"), e.get("end_id"))
        if key in key_to_idx:
            merged[key_to_idx[key]] = e
            replaced += 1
        else:
            merged.append(e)
            added += 1

    print(f"[INFO] Replaced {replaced} existing edges")
    print(f"[INFO] Added {added} new edges")
    return merged


def print_summary(edges):
    total = len(edges)
    ok_cnt = sum(1 for e in edges if e.get("status") == "ok")
    pending_cnt = sum(1 for e in edges if e.get("status") != "ok")

    print(f"[INFO] Final edge count: {total}")
    print(f"[INFO] status=ok: {ok_cnt}")
    print(f"[INFO] status!=ok: {pending_cnt}")

    by_map = {}
    for e in edges:
        m = e.get("map_name", "unknown")
        by_map[m] = by_map.get(m, 0) + 1

    print("[INFO] Edge counts by map:")
    for k, v in sorted(by_map.items()):
        print(f"  - {k}: {v}")


# =========================================================
# Main
# =========================================================
def main():
    parser = argparse.ArgumentParser(
        description="Merge Theta* edge summaries into a topology edge cache."
    )
    parser.add_argument("base_cache", type=Path)
    parser.add_argument("platform_summary", type=Path)
    parser.add_argument("floor2_summary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    base_edges = load_json(args.base_cache)
    if not isinstance(base_edges, list):
        raise ValueError("Base edge cache must be a list.")

    # platform -> floor=1
    platform_edges = summary_to_edges(
        args.platform_summary,
        default_map_name="platform",
        default_floor=1,
    )

    # floor2 -> floor=2
    floor2_edges = summary_to_edges(
        args.floor2_summary,
        default_map_name="floor2",
        default_floor=2,
    )

    merged = merge_edges(base_edges, platform_edges + floor2_edges)

    save_json(merged, args.output)
    print_summary(merged)

    print("\n[DONE] Saved merged edge cache:")
    print(args.output)


if __name__ == "__main__":
    main()
