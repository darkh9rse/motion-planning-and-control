# Transformation Drone AMR for Warehouse Operations
## Motion Planning and Control - Course Project

### Project Overview
An **Autonomous Mobile Robot (AMR)** inspired by transformation drones, designed for warehouse cargo pickup and delivery. The robot features:

- **Big spoked wheels** (transformation-drone style) for all-terrain mobility
- **Bottom-hinged gate/ramp** that folds down to create an inclined loading surface
- **Linear actuator with gripper fingers** mounted horizontally through the chassis walls — extends out to grip cargo, retracts to pull it onto the ramp
- **Internal conveyor belt** to move cargo deeper inside the chassis
- **Compact low-profile chassis** sitting between oversized wheels

The system includes a full **motion planning pipeline**: configuration space generation, skeleton roadmap extraction, and Dijkstra-based path planning with automatic shape-mode switching for navigating warehouse aisles.

---

### Mechanism (How It Works)

1. **Approach** — The AMR drives to the cargo location
2. **Lower Ramp** — The front gate hinges open at the bottom, folding down to create an inclined ramp (tip touches the ground)
3. **Extend Actuator** — Linear actuator pushes gripper fingers out horizontally through the chassis front wall
4. **Grip** — Fingers (initially open wider than the cargo) close inward to clamp the box
5. **Retract & Pull** — Actuator retracts, dragging the cargo up the inclined ramp into the chassis
6. **Close Gate** — Ramp folds back up, sealing the cargo inside
7. **Conveyor** — Internal belt moves the cargo deeper into the chassis
8. **Deliver** — AMR drives to drop-off, opens ramp, releases fingers, cargo slides out

---

### File Structure

```
├── main_pipeline.m          # Master script — runs the full motion planning pipeline
├── warehouse_environment.m  # Defines the warehouse layout (shelves, obstacles)
├── morphing_robot.m         # Robot model with 4 shape modes for path planning
├── generate_cspace.m        # Configuration space generator (2D Minkowski + 3D sampled)
├── generate_skeleton.m      # Skeleton / medial axis extraction for roadmap
├── plan_path.m              # Dijkstra path planner on skeleton graph
├── visualize_all.m          # Multi-panel visualization (warehouse, C-space, skeleton, path)
├── animate_robot.m          # Robot animation along planned path
├── analyze_morphing.m       # Compare C-spaces across all shape modes
├── robot_skeleton.m         # 3D animated skeleton of the physical AMR mechanism
└── README.md                # This file
```

### How to Run

**Motion Planning Pipeline:**
```matlab
main_pipeline
```
Runs all steps (warehouse → C-space → skeleton → path → visualize → animate).

**3D Robot Mechanism Animation (standalone):**
```matlab
robot_skeleton
```
Shows the full pickup-and-delivery sequence with animated ramp, actuator, fingers, and conveyor.

---

### Module Descriptions

| Module | Purpose |
|--------|---------|
| `warehouse_environment` | 20m × 15m warehouse with 4 rows of shelves, pillars, and 2m aisles. Returns obstacle list and binary occupancy grid. |
| `morphing_robot` | Robot with 4 modes: **default** (0.8×0.8), **narrow** (0.4×1.2), **flat** (1.2×0.4), **compact** (0.5 dia). Used for C-space and path planning. |
| `generate_cspace` | Builds 2D C-space via Minkowski sum (circular dilation) and 3D C-space (x, y, θ) by polygon collision sampling. |
| `generate_skeleton` | Extracts medial axis of free C-space using Zhang-Suen thinning. Builds sparse adjacency graph for planning. |
| `plan_path` | Dijkstra shortest path on skeleton graph with clearance-aware edge weights. Outputs per-waypoint morphing plan. |
| `visualize_all` | 4-panel figure: warehouse layout, C-space (rectangle-based), distance transform + skeleton, planned path. |
| `animate_robot` | Smooth animation of robot traversing the planned path, switching shape modes in real-time. |
| `analyze_morphing` | Compares free-space % and clearance statistics across all 4 shape modes. |
| `robot_skeleton` | **Standalone** 3D animated skeleton of the physical AMR — shows wheels, ramp, linear actuator, finger grip/release, conveyor, cargo pickup & drop-off. |

### Physical Robot Features (robot_skeleton.m)

| Component | Description |
|-----------|-------------|
| **Chassis** | Compact low-profile slab between 4 large wheels |
| **Wheels** | Transformation-drone style — large diameter (bigger than body height), 6 spokes, hub caps |
| **Gate / Ramp** | Bottom-hinged at front face; swings outward+down so tip reaches ground; creates inclined loading surface |
| **Linear Actuator** | Horizontal piston mounted through chassis walls; extends/retracts to push/pull gripper fingers |
| **Gripper Fingers** | Two prongs at actuator rod ends; open wide → close to grip cargo → open to release |
| **Conveyor Belt** | Dark strip on chassis floor with rollers; moves cargo deeper inside after loading |

### Performance Notes
- **2D C-space** at 5cm resolution: fast (~seconds)
- **3D C-space** uses coarser 20cm grid to stay feasible
- **Skeleton thinning** can be slow on fine grids — set `env.resolution = 0.2` for faster testing
- **No external toolboxes required** (Image Processing Toolbox NOT needed)
- All algorithms implemented from scratch (dilation, distance transform, thinning, Dijkstra)

### Key Concepts
- **Configuration Space (C-space)**: Workspace obstacles expanded by robot footprint via Minkowski sum
- **Skeleton / Medial Axis**: Topological roadmap maximizing clearance from obstacles (Zhang-Suen thinning)
- **Morphing**: Robot adapts shape mode based on local clearance along planned path
- **Transformation Drone AMR**: Physical mechanism with ramp + actuator + fingers for autonomous cargo handling
