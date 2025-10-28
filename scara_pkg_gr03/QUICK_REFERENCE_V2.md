# Quick Reference - SCARA V2 System

## Launch Order

### 1. Start SCARA Digital Twin (RViz + Kinematics)
```bash
ros2 launch scara_pkg_gr03 scara_digital_twin_v2.launch.py
```
**Includes:** RViz, Robot State Publisher, Forward/Inverse Kinematics, Twist Mux

### 2. Start Trajectory Execution (DXF + Planner)
```bash
ros2 launch scara_pkg_gr03 dxf_exporter_v2.launch.py
```
**Includes:** DXF Exporter V2, Trajectory Planner V2

---

## Configuration File
Edit: `config/scara_v2_params.yaml`

### Quick Parameters to Adjust:
- `dxf_file`: Which DXF to process
- `workspace_center_x/y`: Where to position the piece (mm)
- `scale`: Size adjustment (1.0 = 100%, 0.5 = 50%)
- `safe_height`: Z position for transitions (mm)
- `prismatic_down_height`: Z position during work (mm)
- `publish_rate`: How fast to publish waypoints (Hz)
- `min_distance_mm`: Minimum distance to filter redundant points (mm)

---

## Key Topics

### Published by DXF Exporter:
- `/dxf_pointcloud` - Original waypoints
- `/dxf_figures_info` - Figure metadata

### Published by Trajectory Planner:
- `/desired_pos` (Twist) - Target positions at 20Hz
  - `.linear.x` = X position (mm)
  - `.linear.y` = Y position (mm)
  - `.linear.z` = Z position (mm)
- `/planned_trajectory_rviz` (Path) - Full trajectory for visualization (meters)

### Consumed by Inverse Kinematics:
- `/desired_pos` → converts to joint angles → `/scara_conf`

---

## Common Adjustments

### Make it faster:
```yaml
time_work_segment: 5.0
time_xy_transition: 2.0
publish_rate: 30.0
```

### Make it smoother:
```yaml
interpolation_type: "quintic"
time_work_segment: 30.0
min_distance_mm: 0.5
publish_rate: 50.0
```

### Scale and reposition piece:
```yaml
workspace_center_x: 150.0  # Move to different location
workspace_center_y: 100.0
scale: 0.7                  # Make it 70% size
```

---

## Files Summary

**Launch Files:**
- `scara_digital_twin_v2.launch.py` - Visualization + Kinematics
- `dxf_exporter_v2.launch.py` - DXF Processing + Trajectory

**Nodes:**
- `dxf_exporter_node_v2` - Parses DXF, applies transforms, publishes waypoints
- `scara_trajectory_planner_v2` - Generates smooth trajectory with interpolation

**Config:**
- `config/scara_v2_params.yaml` - All parameters in one file
