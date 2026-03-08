# Morphing Robot for Warehouse Operations
## Motion Planning and Control - Course Project

### Project Overview
A morphing robot that can change its shape (default, narrow, flat, compact) to navigate through a warehouse environment with shelves, aisles, and obstacles. The system generates configuration spaces, extracts skeleton roadmaps, and plans optimal paths with automatic morphing decisions.

---

### File Structure

```
├── main_pipeline.m          # Master script - runs the full pipeline
├── warehouse_environment.m  # Defines the warehouse layout (shelves, obstacles)
├── morphing_robot.m         # Morphing robot model (4 shape modes)
├── generate_cspace.m        # Configuration space generator (2D + 3D)
├── generate_skeleton.m      # Skeleton/medial axis extraction for roadmap
├── plan_path.m              # Dijkstra path planner on skeleton graph
├── visualize_all.m          # Multi-panel visualization
├── animate_robot.m          # Robot animation along planned path
├── analyze_morphing.m       # Compare C-spaces across all morphing modes
└── README.md                # This file
```

### How to Run

1. Open MATLAB
2. Navigate to this folder (`cd` to the project directory)
3. Run:
   ```matlab
   main_pipeline
   ```
4. The script will run all steps sequentially and produce figures.

### Module Descriptions

| Module | Purpose |
|--------|---------|
| `warehouse_environment` | 20m × 15m warehouse with 4 rows of shelves, pillars, and 2m aisles. Returns a struct with obstacle list and binary occupancy grid. |
| `morphing_robot` | Robot with 4 modes: **default** (0.8×0.8), **narrow** (0.4×1.2), **flat** (1.2×0.4), **compact** (0.5 dia circle). Each mode has different speed and footprint. |
| `generate_cspace` | Builds 2D C-space via Minkowski sum dilation and 3D C-space (x, y, θ) by sampling. |
| `generate_skeleton` | Extracts medial axis of free C-space using Zhang-Suen thinning. Builds a graph for path planning. |
| `plan_path` | Dijkstra shortest path on skeleton graph with clearance-aware weighting. Outputs morphing plan. |
| `visualize_all` | 4-panel figure: warehouse, C-space, distance+skeleton, planned path. |
| `animate_robot` | Smooth animation of robot traversing the path, morphing shape in real-time. |
| `analyze_morphing` | Compares free-space percentage and clearance across all 4 morphing modes. |

### Performance Notes
- The **2D C-space** at 5cm resolution is fast (~seconds)
- The **3D C-space** uses a coarser grid (20cm) to keep computation feasible
- The **skeleton thinning** can be slow on fine grids. Set `env.resolution = 0.1` for faster testing.
- No external toolboxes required (Image Processing Toolbox NOT needed)

### Key Concepts
- **Configuration Space (C-space)**: Workspace obstacles expanded by robot footprint
- **Minkowski Sum**: Used for C-space obstacle expansion
- **Skeleton/Medial Axis**: Topological roadmap maximizing clearance from obstacles
- **Morphing**: Robot adapts shape based on local clearance along the path
