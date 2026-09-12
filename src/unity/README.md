# Unity Research Code

This directory contains only selected C# scripts written for the Unity-based multi-floor delivery system. It is not a Unity project: scenes, prefabs, materials, models, textures, packages, and Asset Store content are intentionally excluded. Source presence documents the implementation scope but does not establish that a complete runtime was built or tested from this public subset.

## Modules

- `Robot/` contains the Rigidbody-based Pure Pursuit route follower and critical-waypoint handling used near vertical transfers.
- `Navigation/` reads scene-defined fixed routes and executes generated CSV waypoint and wait commands.
- `Topology/` defines floor surfaces, task/depot zones, semantic nodes, and graph search.
- `Elevator/` contains the trigger events and two-level elevator state machine.
- `Delivery/` generates randomized delivery regions A, B, and C plus the unload region.
- `Sensors/` implements multi-height planar LiDAR raycasts.
- `Integration/` sends LiDAR packets to Python and exports scene nodes/targets for the planning pipeline.

The source project records Unity `6000.3.2f1`. The included scripts reference UnityEngine APIs, including Rigidbody physics; they do not reference the excluded ML-Agents prototype. The source project also used render-pipeline packages for its scene, but those assets and package caches are not distributed here.

PARS local recovery and PP/PARS switching are implemented in the separate Python research environment. No Unity runtime bridge for SAC takeover was located in the reviewed C# source, so this subset does not claim a live Unity-to-policy control handoff. That scoped finding does not establish whether such a bridge existed elsewhere historically.
