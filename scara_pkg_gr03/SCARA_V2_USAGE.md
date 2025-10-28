# SCARA V2 Trajectory System - Quick Start Guide

## Overview
The V2 trajectory system provides enhanced DXF-based trajectory planning with:
- Automatic workspace centering and scaling
- Optimized entity ordering (nearest-neighbor)
- 20Hz publishing rate with 1mm filtering
- Configurable prismatic joint heights
- RViz trajectory visualization

## Files Created
1. **Nodes:**
   - `scara_trajectory_planner_v2.py` - Trajectory planner with 20Hz rate and filtering
   - `dxf_exporter_node_v2.py` - DXF parser with scaling and centering

2. **Launch Files:**
   - `scara_digital_twin_v2.launch.py` - SCARA visualization and kinematics (RViz, FK, IK, etc.)
   - `dxf_exporter_v2.launch.py` - DXF exporter + trajectory planner together

3. **Configuration:**
   - `config/scara_v2_params.yaml` - All parameters for both nodes

## Two-Step Workflow

### Step 1: Launch SCARA Digital Twin
In **Terminal 1**, launch the SCARA visualization and kinematics:

```bash
cd ~/ros2_ws_2502
source install/setup.bash
ros2 launch scara_pkg_gr03 scara_digital_twin_v2.launch.py
```

This launches:
- RViz2 with robot visualization
- Forward kinematics node
- Inverse kinematics node
- Twist mux
- Goal pose translator

### Step 2: Launch DXF Exporter + Trajectory Planner
In **Terminal 2**, launch the DXF processing and trajectory generation:

```bash
cd ~/ros2_ws_2502
source install/setup.bash
ros2 launch scara_pkg_gr03 dxf_exporter_v2.launch.py
```

**Expected Output:**
```
[INFO] [dxf_exporter_node_v2]: === DXF Exporter Node V2 Configuration ===
[INFO] [dxf_exporter_node_v2]: DXF file: .../prueba_7_8.dxf
[INFO] [dxf_exporter_node_v2]: Target workspace center: (200.0, 0.0) mm
[INFO] [dxf_exporter_node_v2]: Scale factor: 1.0
[INFO] [dxf_exporter_node_v2]: 📄 Parsing DXF entities...
[INFO] [dxf_exporter_node_v2]:   ✓ POLYLINE: 5 points
[INFO] [dxf_exporter_node_v2]: 📍 Original DXF center: (195.00, -25.00) mm
[INFO] [dxf_exporter_node_v2]: ✅ Final center: (200.00, 0.00) mm
[INFO] [dxf_exporter_node_v2]: 🔄 Sorting entities using nearest-neighbor algorithm...
[INFO] [dxf_exporter_node_v2]: 📤 Published PointCloud: 42 points
[INFO] [dxf_exporter_node_v2]: 📤 Published figures info: 3 figures

[INFO] [scara_trajectory_planner_v2]: === SCARA Trajectory Planner V2 Configuration ===
[INFO] [scara_trajectory_planner_v2]: Interpolation type: quintic
[INFO] [scara_trajectory_planner_v2]: Safe height (transitions): 20.0 mm
[INFO] [scara_trajectory_planner_v2]: Work height (path): 10.0 mm
[INFO] [scara_trajectory_planner_v2]: Publish rate: 20.0 Hz
[INFO] [scara_trajectory_planner_v2]: Min distance filter: 1.0 mm
[INFO] [scara_trajectory_planner_v2]: 📥 Received 42 points in dxf_pointcloud.
[INFO] [scara_trajectory_planner_v2]: 🔧 Generating complete trajectory...
[INFO] [scara_trajectory_planner_v2]: ✅ Trajectory generation complete: 8361 waypoints
[INFO] [scara_trajectory_planner_v2]: 📐 Published trajectory to RViz: 8361 poses
[INFO] [scara_trajectory_planner_v2]: 📍 Publishing waypoint 0/8361: (165.00, -25.00, 20.00)
```

The trajectory planner will automatically start publishing waypoints to `/desired_pos` which
the inverse kinematics node (running in Terminal 1) will process and send to the robot.

## Summary

**Terminal 1:** SCARA Digital Twin (RViz + Kinematics)
```bash
ros2 launch scara_pkg_gr03 scara_digital_twin_v2.launch.py
```

**Terminal 2:** DXF Processing + Trajectory Execution
```bash
ros2 launch scara_pkg_gr03 dxf_exporter_v2.launch.py
```

## Configuration

Edit `config/scara_v2_params.yaml` to customize parameters:

### DXF Exporter Parameters
```yaml
dxf_exporter_node_v2:
  ros__parameters:
    dxf_file: "prueba_7_8.dxf"          # DXF filename or absolute path
    workspace_center_x: 200.0            # Target X center (mm)
    workspace_center_y: 0.0              # Target Y center (mm)
    scale: 1.0                           # Scale factor (1.0 = 100%)
```

### Trajectory Planner Parameters
```yaml
scara_trajectory_planner_v2:
  ros__parameters:
    interpolation_type: "quintic"        # "linear", "cubic", or "quintic"
    safe_height: 20.0                    # Z height for transitions (mm)
    prismatic_down_height: 10.0          # Z height during path (mm)
    time_work_segment: 20.0              # Time per segment (s)
    time_segment_close: 20.0             # Time to close figure (s)
    time_prismatic_up: 0.5               # Time to raise tool (s)
    time_prismatic_down: 0.5             # Time to lower tool (s)
    time_xy_transition: 20.0             # Time between figures (s)
    publish_rate: 20.0                   # Publishing rate (Hz)
    min_distance_mm: 1.0                 # Min distance to publish (mm)
```

## Common Use Cases

### 1. Small piece in corner of workspace (scaled down)
```yaml
dxf_exporter_node_v2:
  ros__parameters:
    dxf_file: "small_part.dxf"
    workspace_center_x: 150.0
    workspace_center_y: 150.0
    scale: 0.5                           # Half size
```

### 2. Fast execution (shorter times)
```yaml
scara_trajectory_planner_v2:
  ros__parameters:
    interpolation_type: "linear"
    time_work_segment: 5.0
    time_segment_close: 2.0
    time_xy_transition: 3.0
    publish_rate: 10.0
```

### 3. High-quality smooth motion
```yaml
scara_trajectory_planner_v2:
  ros__parameters:
    interpolation_type: "quintic"
    time_work_segment: 30.0
    publish_rate: 50.0
    min_distance_mm: 0.5
```

## Topics Published

### From DXF Exporter:
- `/dxf_pointcloud` (PointCloud) - All waypoints
- `/dxf_figures_info` (String) - Figure metadata (JSON)

### From Trajectory Planner:
- `/desired_pos` (Twist) - Target positions (mm) at 20Hz
  - `linear.x` = X position
  - `linear.y` = Y position
  - `linear.z` = Z position (prismatic joint)
- `/planned_trajectory_rviz` (Path) - Full trajectory for RViz visualization (meters)

## Troubleshooting

### Issue: Trajectory planner doesn't receive data
**Solution:** The DXF exporter and trajectory planner are now in the same launch file. If you see no trajectory, check that the DXF file path is correct in the config.

### Issue: Robot doesn't move in RViz
**Solution:** Make sure you launched the digital twin FIRST (Terminal 1), then the DXF exporter (Terminal 2).

### Issue: Piece is outside workspace
**Solution:** Adjust `workspace_center_x/y` parameters in config file.

### Issue: Trajectory is too fast/slow
**Solution:** Modify `time_work_segment` and `publish_rate` parameters.

### Issue: Too many waypoints published
**Solution:** Increase `min_distance_mm` parameter (e.g., 2.0 or 5.0).

## Visualization in RViz

Add these displays in RViz:
1. **Path** display:
   - Topic: `/planned_trajectory_rviz`
   - Color: Blue
   - Shows complete trajectory

2. **PointCloud** display:
   - Topic: `/dxf_pointcloud`
   - Color: Red
   - Shows original DXF waypoints

## Next Steps

After launching, you can:
1. Monitor trajectory execution: `ros2 topic echo /desired_pos`
2. View trajectory in RViz
3. Adjust parameters in `scara_v2_params.yaml` and relaunch
4. Try different DXF files by changing the `dxf_file` parameter
