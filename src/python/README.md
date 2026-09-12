# Python Research Code

The Python side contains the smallest source subset needed to inspect the planning pipeline and the separate PARS local-recovery research environment. It does not include maps, task workbooks, routes, trained policies, logs, or historical environments. The retained Python source does not establish a live SAC-to-Unity control bridge.

## Modules

- `mapping/` receives Unity multi-height LiDAR packets, accumulates a log-odds occupancy grid, and exports NPZ plus ROS-style PGM/YAML maps.
- `topology/` builds fixed, transfer, and pending Theta* edges; merges planned edge geometry; and derives task-node distance tables.
- `planning/` implements occupancy inflation and Theta* any-angle path planning.
- `optimization/` implements the CBC multi-trip, multi-commodity CVRP formulation.
- `navigation/` expands optimized task trips and cached topology edges into a Unity waypoint CSV/JSON stream.
- `pars/` contains map/path utilities, the PARS hybrid recovery environment, its Gymnasium adapter, and a model-evaluation entry point.

## Planning pipeline and data interfaces

The Unity-supporting data path is mapping → topology cache → Theta* → edge merge → distance table → CVRP → waypoint generation. Major inputs are LiDAR UDP packets, occupancy-map NPZ files, scene-node/edge tables, route summaries, and task demands. Major outputs are maps, cached edge geometry, task-distance tables, optimized trip sequences, and a waypoint stream that the retained Unity loader can read.

Separately, the PARS module accepts an occupancy map, reference routes, and a locally supplied SAC policy. Within its Python environment it implements nominal Pure Pursuit, blocked-path switching to local velocity actions, a locked forward rejoin point, and return to nominal tracking. No retained Python or C# interface sends those SAC actions to the Unity Rigidbody controller. The source documents these code paths; it does not by itself establish completed runtime testing.

Theta*, Dijkstra search, CBC, CVRP, Pure Pursuit, Gymnasium, and Soft Actor-Critic are established methods or tools. Project-specific work is the multi-trip/multi-commodity delivery formulation, multi-floor data interfaces, route-expansion logic, and path-attentive switching/rejoin design in PARS.

Exact historical dependency versions were not preserved. See `requirements-minimal.txt` for unpinned core dependencies.
