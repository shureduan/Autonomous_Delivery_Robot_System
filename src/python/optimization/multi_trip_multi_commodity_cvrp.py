import math
from pathlib import Path
from typing import Dict, Tuple, List
import pandas as pd
from ortools.linear_solver import pywraplp


# =========================================================
# Excel parsing (Format B)
# =========================================================
def _norm(x) -> str:
    return str(x).strip()


def read_distance_matrix_format_b(df: pd.DataFrame) -> Tuple[List[str], Dict[Tuple[str, str], int]]:
    raw = df.copy()
    node_col = raw.columns[0]
    row_nodes = [_norm(x) for x in raw[node_col].tolist()]
    col_nodes = [_norm(c) for c in raw.columns[1:].tolist()]

    if set(row_nodes) != set(col_nodes):
        raise ValueError(
            f"Distance matrix invalid: row nodes {row_nodes} and column nodes {col_nodes} do not match."
        )

    mat = raw.drop(columns=[node_col])
    mat.columns = col_nodes

    dist = {}
    for i, u in enumerate(row_nodes):
        for j, v in enumerate(col_nodes):
            val = mat.iloc[i, j]
            if pd.isna(val):
                raise ValueError(f"Missing distance at row {u}, col {v}.")
            dist[(u, v)] = int(round(float(val)))

    for (u, v), c in list(dist.items()):
        if (v, u) not in dist:
            dist[(v, u)] = c

    return col_nodes, dist


def read_demands_sheet(df: pd.DataFrame) -> Tuple[List[str], Dict[str, Dict[str, int]]]:
    d0 = df.copy()
    d0.columns = [str(c).strip().lower() for c in d0.columns]

    node_col = None
    for cand in ["node", "id", "name", "point"]:
        if cand in d0.columns:
            node_col = cand
            break
    if node_col is None:
        node_col = d0.columns[0]

    for g in ["x", "y", "z"]:
        if g not in d0.columns:
            raise ValueError("Demand sheet must contain columns x, y, z.")

    nodes = []
    demands_by_good = {g: {} for g in ["x", "y", "z"]}

    for _, row in d0.iterrows():
        n = _norm(row[node_col])
        if n == "" or n.lower() == "nan":
            continue

        nodes.append(n)

        for g in ["x", "y", "z"]:
            val = row[g]
            if pd.isna(val):
                val = 0
            demands_by_good[g][n] = int(round(float(val)))

    return nodes, demands_by_good


def infer_depot(nodes: List[str], demands_by_good: Dict[str, Dict[str, int]]) -> str:
    # 优先使用正式命名的 Depot
    if "Depot" in nodes:
        return "Depot"

    # 兼容旧文件：找三类需求都为 0 的点
    for n in nodes:
        if (
            demands_by_good["x"].get(n, 0) == 0
            and demands_by_good["y"].get(n, 0) == 0
            and demands_by_good["z"].get(n, 0) == 0
        ):
            return n

    # 再兼容特别旧的表
    if "A" in nodes:
        return "A"

    return nodes[0]


# =========================================================
# CBC MIP (multi-commodity, split delivery, explicit trips)
# =========================================================
def solve_mip_cbc_multi_goods(
    nodes: List[str],
    depot: str,
    dist: Dict[Tuple[str, str], int],
    demands_by_good: Dict[str, Dict[str, int]],
    Q: int = 50,
    time_limit_sec: int = 60,
) -> Tuple[float, List[dict]]:
    goods = ["x", "y", "z"]
    customers = [n for n in nodes if n != depot]

    total_demand_all = sum(demands_by_good[g].get(i, 0) for g in goods for i in customers)
    if total_demand_all == 0:
        return 0.0, []

    Kmin = math.ceil(total_demand_all / Q)
    Kmax = Kmin + 2 * len(customers)

    nC = len(customers)

    solver = pywraplp.Solver.CreateSolver("CBC_MIXED_INTEGER_PROGRAMMING")
    if solver is None:
        raise RuntimeError("CBC solver not available in your OR-Tools installation.")
    solver.SetTimeLimit(time_limit_sec * 1000)

    # ---------- Vars ----------
    y = {k: solver.BoolVar(f"y[{k}]") for k in range(Kmax)}

    x = {}
    for k in range(Kmax):
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                x[(k, i, j)] = solver.BoolVar(f"x[{k},{i},{j}]")

    v = {}
    for k in range(Kmax):
        for c in customers:
            v[(k, c)] = solver.BoolVar(f"v[{k},{c}]")

    f = {}
    for k in range(Kmax):
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                for g in goods:
                    f[(k, i, j, g)] = solver.IntVar(0, Q, f"f[{k},{i},{j},{g}]")

    deliv = {}
    for k in range(Kmax):
        for c in customers:
            for g in goods:
                ub = int(demands_by_good[g].get(c, 0))
                deliv[(k, c, g)] = solver.IntVar(0, ub, f"deliv[{k},{c},{g}]")

    u = {}
    for k in range(Kmax):
        for c in customers:
            u[(k, c)] = solver.NumVar(0.0, float(nC), f"u[{k},{c}]")

    # ---------- Constraints ----------
    for k in range(Kmax):
        # depot: leave once if used
        solver.Add(sum(x[(k, depot, j)] for j in nodes if j != depot) == y[k])
        # depot: return once if used
        solver.Add(sum(x[(k, j, depot)] for j in nodes if j != depot) == y[k])

        # customers degree balance and link to v
        for c in customers:
            solver.Add(sum(x[(k, c, j)] for j in nodes if j != c) == v[(k, c)])
            solver.Add(sum(x[(k, j, c)] for j in nodes if j != c) == v[(k, c)])

        # per-arc capacity
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                solver.Add(sum(f[(k, i, j, g)] for g in goods) <= Q * x[(k, i, j)])

        # per-trip capacity
        solver.Add(sum(deliv[(k, c, g)] for c in customers for g in goods) <= Q * y[k])

        # delivery only if visited
        for c in customers:
            for g in goods:
                ub = int(demands_by_good[g].get(c, 0))
                solver.Add(deliv[(k, c, g)] <= ub * v[(k, c)])

        # per-trip commodity conservation
        for g in goods:
            for c in customers:
                inflow = sum(f[(k, i, c, g)] for i in nodes if i != c)
                outflow = sum(f[(k, c, j, g)] for j in nodes if j != c)
                solver.Add(inflow - outflow == deliv[(k, c, g)])

            depot_out = sum(f[(k, depot, j, g)] for j in nodes if j != depot)
            depot_in = sum(f[(k, i, depot, g)] for i in nodes if i != depot)
            solver.Add(depot_out - depot_in == sum(deliv[(k, c, g)] for c in customers))

        # MTZ
        for c in customers:
            solver.Add(u[(k, c)] <= nC * v[(k, c)])

        for i in customers:
            for j in customers:
                if i == j:
                    continue
                solver.Add(
                    u[(k, i)] - u[(k, j)] + nC * x[(k, i, j)]
                    <= (nC - 1) + nC * (1 - v[(k, j)])
                )

    # total demand satisfaction
    for c in customers:
        for g in goods:
            demand_cg = int(demands_by_good[g].get(c, 0))
            solver.Add(sum(deliv[(k, c, g)] for k in range(Kmax)) == demand_cg)

    # symmetry breaking
    for k in range(Kmax - 1):
        solver.Add(y[k] >= y[k + 1])

    # ---------- Objective ----------
    objective = solver.Objective()
    for k in range(Kmax):
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                objective.SetCoefficient(x[(k, i, j)], dist[(i, j)])
    objective.SetMinimization()

    status = solver.Solve()
    if status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        raise RuntimeError("No feasible solution found by CBC. Check distances/demands.")

    obj_val = objective.Value()

    # ---------- Extract trips ----------
    trips = []
    for k in range(Kmax):
        if y[k].solution_value() < 0.5:
            continue

        next_of = {}
        for i in nodes:
            for j in nodes:
                if i == j:
                    continue
                if x[(k, i, j)].solution_value() > 0.5:
                    next_of[i] = j

        route = [depot]
        cur = depot
        guard = 0
        while True:
            guard += 1
            if guard > 100:
                break
            nxt = next_of.get(cur, None)
            if nxt is None:
                break
            route.append(nxt)
            cur = nxt
            if cur == depot:
                break

        deliver = {c: {g: 0 for g in goods} for c in customers}
        for c in customers:
            for g in goods:
                deliver[c][g] = int(round(deliv[(k, c, g)].solution_value()))

        trip_dist = 0
        for a, b in zip(route, route[1:]):
            trip_dist += dist[(a, b)]

        load_total = sum(deliver[c][g] for c in customers for g in goods)

        trips.append({
            "trip_id": len(trips) + 1,
            "route": route,
            "distance": trip_dist,
            "load_total": load_total,
            "deliver": deliver
        })

    return obj_val, trips


# =========================================================
# End-to-end runner + Excel output
# =========================================================
def solve_from_excel_cbc(
    excel_path: str,
    capacity: int = 50,
    time_limit_sec: int = 60,
    output_path: str = "solution_output.xlsx",
    dist_sheet: str | None = None,
    demand_sheet: str | None = None,
):
    xls = pd.ExcelFile(excel_path)
    if len(xls.sheet_names) < 2:
        raise ValueError("Excel must contain at least 2 sheets: distance matrix + demands.")

    dist_sheet = dist_sheet or xls.sheet_names[0]
    demand_sheet = demand_sheet or xls.sheet_names[1]

    df_dist = pd.read_excel(excel_path, sheet_name=dist_sheet)
    df_dem = pd.read_excel(excel_path, sheet_name=demand_sheet)

    nodes_dist, dist = read_distance_matrix_format_b(df_dist)
    nodes_dem, demands_by_good = read_demands_sheet(df_dem)

    if not set(nodes_dem).issubset(set(nodes_dist)):
        missing = sorted(set(nodes_dem) - set(nodes_dist))
        raise ValueError(f"Demand nodes not in distance matrix: {missing}")

    nodes = nodes_dem[:]
    depot = infer_depot(nodes, demands_by_good)

    print("Detected nodes:", nodes)
    print("Detected depot:", depot)

    for i in nodes:
        for j in nodes:
            if i == j:
                continue
            if (i, j) not in dist:
                raise ValueError(f"Missing distance ({i},{j})")

    obj, trips = solve_mip_cbc_multi_goods(
        nodes=nodes,
        depot=depot,
        dist=dist,
        demands_by_good=demands_by_good,
        Q=capacity,
        time_limit_sec=time_limit_sec
    )

    goods = ["x", "y", "z"]
    customers = [n for n in nodes if n != depot]

    trip_rows = []
    for t in trips:
        row = {
            "trip_id": t["trip_id"],
            "route": "->".join(t["route"]),
            "distance": t["distance"],
            "load_total": t["load_total"],
        }
        for c in customers:
            for g in goods:
                row[f"deliver_{c}_{g}"] = t["deliver"][c][g]
        trip_rows.append(row)

    df_trips = pd.DataFrame(trip_rows)
    df_summary = pd.DataFrame([{
        "depot": depot,
        "capacity_Q": capacity,
        "time_limit_sec": time_limit_sec,
        "num_nodes": len(nodes),
        "num_customers": len(customers),
        "optimal_total_distance": obj,
        "num_trips_used": len(trips),
        "status_note": "If solver status is OPTIMAL, this is a proven global optimum. FEASIBLE means best found within time limit."
    }])

    with pd.ExcelWriter(output_path, engine="openpyxl") as writer:
        df_summary.to_excel(writer, index=False, sheet_name="summary")
        df_trips.to_excel(writer, index=False, sheet_name="trip_sequence")

    print("=== CBC MIP SOLVED (FIXED) ===")
    print("Depot:", depot)
    print("Capacity Q:", capacity)
    print("Optimal total distance:", obj)
    print("Trips used:", len(trips))
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    repository_root = Path(__file__).resolve().parents[3]
    solve_from_excel_cbc(
        excel_path=str(repository_root / "data" / "examples" / "task_input.xlsx"),
        capacity=50,
        time_limit_sec=60,
        output_path=str(repository_root / "outputs" / "solution_output.xlsx")
    )
