# Autonomous Delivery Robot System

*Independent research on multi-floor autonomous material delivery for construction-site logistics.*

The project combines a Unity-based multi-floor delivery pipeline with a separate Python research implementation of PARS for local recovery. The available source documents both components, but does not establish a live SAC-to-Unity control bridge.

The Unity component covers sensing, topology and target export, generated-path loading, Rigidbody-based Pure Pursuit, elevator motion, and delivery dwell logic. The supporting Python pipeline covers mapping, topology processing, Theta* planning, CVRP scheduling, and waypoint generation. Separately, the Python PARS environment implements blocked-path recovery, PP/PARS switching, and a locked path-rejoin target. Source presence documents these implementations and interfaces; it is not evidence that the full combined runtime was tested.

## Research Problem

Construction delivery combines several decisions that are often studied separately. A robot has limited carrying capacity, may need several depot-return trips, must transport multiple material types, and must reach delivery regions connected by ramps or an elevator. During execution, workers, equipment, or stored materials may also block a planned segment. The research question is how to represent these constraints across task scheduling and cross-floor execution while studying local route recovery without replacing the global plan.

## Key Contributions

1. **Multi-trip multi-commodity CVRP formulation.** The task scheduler models one robot making repeated trips from a depot while carrying several material types. Trip-indexed route, activity, unloading, and commodity-flow variables couple each customer visit to loading, capacity, demand fulfillment, and depot return.

2. **Hierarchical multi-floor planning and execution architecture.** Per-floor occupancy maps and a sparse topology connect the depot, delivery regions, ramps, and elevator transfer points. Task routes are expanded through cached graph edges into a continuous waypoint stream that Unity can execute, including cross-floor transitions and dwell events.

3. **Python PARS local-recovery implementation.** The separate research environment implements nominal Pure Pursuit, geometric blockage detection, SAC velocity actions, PP/PARS switching, a locked forward rejoin target, and return to nominal tracking. This is a retained implementation rather than only a proposed idea, but the reviewed source does not establish its live takeover of the Unity controller.

## System Architecture

![Multi-floor system topology](figures/system_topology.png)

The released source documents two related scopes:

```text
Unity sensing and scene export
        ↓
Python mapping → topology → Theta* → distance table → CVRP → waypoints
        ↓
Unity route loading → Pure Pursuit → elevator and delivery waits
```

```text
Occupancy map + reference route + locally supplied SAC policy
        ↓
Python PARS environment: nominal PP ↔ blocked-path recovery → locked rejoin
```

Simulated multi-height LiDAR supports separate occupancy representations for vertically disconnected spaces. Semantic nodes identify locations that affect delivery execution, while graph edges preserve both traversal cost and waypoint geometry. Theta* supplies collision-aware any-angle paths for appropriate same-floor connections; fixed routes represent surveyed segments such as ramps; transfer edges represent elevator movement. The resulting graph distances inform the delivery optimizer, and the selected task order is expanded back into executable geometric paths.

The Unity and planning components exchange LiDAR, scene-node, and waypoint data through retained UDP and file interfaces. The Python PARS module uses compatible map and route concepts, but the reviewed material does not establish a runtime SAC command channel back to Unity. This boundary does not determine whether such a bridge existed elsewhere historically.

Theta*, graph shortest-path search, Pure Pursuit, Soft Actor-Critic, CBC/OR-Tools, Gymnasium, and LiDAR mapping concepts are established methods. The project contribution is their application-specific formulation, retained interfaces, and component-level integration for the studied construction-delivery setting.

## Method

### Task-level optimization

The delivery problem is formulated as a mixed-integer, multi-trip, multi-commodity capacitated vehicle routing problem. The objective minimizes route cost subject to trip activation, depot departure and return, customer visitation, vehicle capacity, commodity-specific demand satisfaction, and route-consistent material flow. Explicit unloading variables connect delivered quantities to customer visits, and subtour-elimination constraints preserve valid trips. The implementation uses CBC through OR-Tools.

### Multi-floor route planning

The environment is divided into floor-specific occupancy maps so that geometry at different heights does not overlap in a single 2-D planning space. A semantic topology links delivery nodes and transition points. Theta* plans selected intra-floor edges after occupancy thresholding and obstacle inflation, and shortest-path search over the combined topology produces task-to-task distances. Once the CVRP solver chooses trip sequences, cached edge paths are concatenated into route waypoints with elevator and delivery waits.

### Local recovery

PARS is implemented as a local recovery layer in the separate Python research environment rather than as a replacement for the global planner. Its observation combines local LiDAR ranges with path-relative goal, return-point, tracking-error, heading, and velocity information. The continuous action specifies linear and angular velocity. A geometric switching rule activates recovery when the nominal path is blocked; once reconnection becomes feasible, the return point is locked to avoid target drift and nominal tracking resumes. Archived materials contain several experiment versions, and this release does not select one as canonical.

## Simulation

The manuscript describes a three-level Unity construction environment with a ground-level depot, two ramps, an elevator, and delivery regions on the upper levels. Delivery regions are randomized within their designated areas. A simulated wheeled robot uses multi-height 2-D LiDAR measurements for mapping and local perception. The simulation is intended to study the interfaces among logistics optimization, cross-floor route execution, and obstacle recovery before physical deployment.

## Representative Outputs

![Representative Theta-star path-planning output](figures/theta_star_path.png)

The figure above is a representative Theta* planning output on an occupancy map. It shows an any-angle route between topology nodes after map processing and obstacle inflation; it is a qualitative example rather than an aggregate performance result.

The Unity summary table in the study manuscript records five evaluation runs with 20 randomized deliveries per run, for 100/100 completed deliveries and a mean round-trip time of 223 s. These manuscript aggregates are not evidence that PARS controlled Unity. Per-delivery records have not been located in the materials reviewed for this release, so the values are not presented as a fully reproducible benchmark.

## My Role

This was conducted as an Independent Study. I independently designed the system architecture, implemented the Python and Unity delivery pipeline, formulated the optimization model, developed the separate PARS recovery environment, designed the simulation experiments, and prepared the manuscript.

## Repository

- `src/` — selected Python and Unity C# research implementation.
- `docs/` — reader-facing source-selection notes and the code pipeline map.
- `figures/` — selected research diagrams and qualitative planning output.
- `configs/` — repository-relative example path conventions.

This is a compact method-inspection release, not a complete runnable archive. Maps, task inputs, the Unity project, trained models, and generated experiment artifacts are not included.

No open-source license is granted for this release.

## Code Overview

The Python planning code handles LiDAR packet ingestion and occupancy mapping, multi-floor topology construction, Theta* planning, task-distance generation, multi-trip CVRP optimization, and waypoint expansion. Unity C# handles simulated sensing, semantic scene nodes, randomized delivery targets, route loading, Rigidbody-based Pure Pursuit execution, elevator interaction, and wait commands. These components have explicit file and UDP interfaces: Unity sends LiDAR and pose packets and exports scene nodes, while Python generates a waypoint stream for Unity to load. The separate Python PARS environment contains local recovery, PP/PARS switching, and locked-rejoin logic; no live SAC-to-Unity control bridge was located in the reviewed source. Full Unity scenes, prefabs, models, textures, and package caches are not distributed because this repository is a research-code portfolio rather than a complete scene release. Exact historical dependency versions were not preserved. See the [Code Pipeline](docs/CODE_PIPELINE.md) for the file-level mapping.

## Limitations

- Evaluation is simulation-only; no physical-robot or sim-to-real validation is included.
- The current architecture and study evaluate a single robot.
- The public reproducibility package is incomplete and does not support end-to-end reproduction.

## Contact

Jingxuan Duan<br>
Carnegie Mellon University<br>
Email: shureduan0912@gmail.com
