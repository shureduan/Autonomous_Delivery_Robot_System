# Code Pipeline and Integration Boundary

This map connects each research stage to the smallest retained implementation. It documents source-level data interfaces and does not claim that the full chain was run from this public subset. The repository does not include the scene, maps, generated tables, trained models, or historical environment needed for end-to-end execution.

## Unity delivery pipeline and planning support

```text
Unity LiDAR and scene-node export
        ↓
Python occupancy map → topology → Theta* → distance matrix → multi-trip CVRP
        ↓
Python waypoint stream
        ↓
Unity route loader → Pure Pursuit → elevator and delivery waits
```

| Stage | Retained source | Input → output |
|---|---|---|
| Unity LiDAR / scene | `src/unity/Sensors/MultiHeightLidar2D.cs`; `src/unity/Integration/LidarUdpSender.cs` | Unity raycasts and robot pose → `LDM3` UDP packets |
| Occupancy map | `src/python/mapping/lidar_occupancy_mapping.py` | LiDAR UDP packets → log-odds occupancy map, NPZ, PGM, and YAML |
| Node-edge topology | `src/unity/Topology/TopoNode.cs`; `NavSurface.cs`; `NavZone.cs`; `TopologyGraphManager.cs`; `src/unity/Integration/SceneNodeCsvExporter.cs`; `src/python/topology/build_edge_cache.py` | Scene nodes, delivery regions, edge definitions, fixed routes → typed multi-floor edge cache |
| Theta* | `src/python/planning/theta_star.py` | Floor map, scene nodes, requested same-floor edges → any-angle waypoint paths and edge summaries |
| Planned-edge merge | `src/python/topology/merge_theta_paths.py` | Base edge cache and Theta* summaries → merged edge cache with geometry |
| Distance matrix | `src/python/topology/build_node_distance_table.py` | Merged topology cache → task-node shortest-path matrix and pairwise route table |
| Multi-trip CVRP | `src/python/optimization/multi_trip_multi_commodity_cvrp.py` | Distance matrix, multi-commodity demands, robot capacity → optimized depot-return trip sequence |
| Waypoint stream | `src/python/navigation/build_combined_waypoints.py` | Trip sequence and merged edge cache → ordered CSV/JSON movement and wait commands |
| Unity route loader | `src/unity/Navigation/UnityRouteExecutor.cs`; `FixedRouteReader.cs` | Generated waypoint CSV or scene route → move/wait command sequence |
| Pure Pursuit | `src/unity/Robot/BodyXYDrive_PurePursuit.cs` | Waypoints and Rigidbody state → nominal force/torque commands |

## Separate Python PARS research module

```text
Occupancy map + reference route + locally supplied SAC policy
        ↓
Python hybrid environment
        ├── nominal Pure Pursuit
        └── blocked route → PARS local velocity actions → locked rejoin → nominal tracking
```

| Stage | Retained source | Input → output |
|---|---|---|
| PARS environment and geometry | `src/python/pars/pars_env.py`; `map_utils.py`; `path_utils.py` | Occupancy map, reference route, and LiDAR-like observations → PP/PARS mode decisions, local state transitions, and locked path-rejoin decisions |
| Gymnasium and policy evaluation | `src/python/pars/pars_gym_env.py`; `evaluate_policy.py` | Locally supplied SAC policy plus map/routes → continuous linear/angular actions and episode summaries inside the Python environment |

## Cross-floor and task support

- `src/unity/Elevator/ElevatorTriggerDetector.cs` and `ElevatorController.cs` implement elevator entry/exit events and platform motion.
- `src/unity/Delivery/DestinationRegionGenerator.cs` creates the randomized delivery regions consumed by scene-node export.
- Waypoint wait tags represent elevator dwell and delivery/unload pauses; the retained code does not include a separate material-manipulation subsystem.

## Integration boundary

The retained Unity code implements LiDAR-to-Python transmission and waypoint execution interfaces. The retained PARS code implements blockage detection, local SAC actions, locked return-point behavior, and re-entry to Pure Pursuit inside the Python research environment. No C# or network bridge that performs SAC takeover of the Unity controller was located in the reviewed source. The components are therefore documented separately rather than as a direct runtime handoff. This scoped finding does not establish whether such a bridge existed elsewhere historically.
