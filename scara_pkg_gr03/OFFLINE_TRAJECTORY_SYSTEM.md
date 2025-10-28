# Offline Trajectory Generation System

## Overview

This system pre-computes all joint configurations before execution, allowing for trajectory validation, optimization, and smoother robot motion. Unlike the online system that computes configurations in real-time, the offline system processes the entire trajectory upfront.

## Architecture

```
DXF File → dxf_exporter_node_v2 → offline_trajectory_generator 
→ offline_inverse_kinematics → joint_trajectory_interpolator 
→ [stored configurations] → trajectory_executor → Pico
```

### Node Pipeline

1. **dxf_exporter_node_v2**: Parses DXF file, applies scaling/centering transformations
2. **offline_trajectory_generator**: Republishes all waypoints with TRANSIENT_LOCAL QoS
3. **offline_inverse_kinematics**: Computes joint configurations for ALL waypoints at once
4. **joint_trajectory_interpolator**: Interpolates J1 and J2 angles (J3 passes through)
5. **trajectory_executor**: Publishes configurations to Pico at controlled rate

## Key Features

- ✅ **Pre-computation**: All IK calculations done before execution
- ✅ **Validation**: Catch IK failures before sending to robot
- ✅ **Selective Interpolation**: Only J1 and J2 are interpolated
- ✅ **No Home Return**: No interpolation between last point and home
- ✅ **Configurable Interpolation**: Linear, cubic, or quintic
- ✅ **Rate Control**: Separate execution rate from computation
- ✅ **Latched Messages**: TRANSIENT_LOCAL QoS for async node startup

## Topics

### Published by Pipeline
- `/dxf_pointcloud` (PointCloud): Raw waypoints from DXF
- `/trajectory_waypoints` (PointCloud): All waypoints for offline processing
- `/joint_configurations` (String/JSON): Raw IK solutions for each waypoint
- `/interpolated_trajectory` (String/JSON): Final interpolated configurations
- `/inv_kin` (Twist): Configurations sent to Pico

### For Visualization
- `/planned_trajectory_rviz` (Path): Planned trajectory visualization
- `/end_effector_path` (Path): Actual robot path

## Configuration Files

### offline_trajectory_params.yaml
```yaml
joint_trajectory_interpolator:
  ros__parameters:
    interpolation_type: "quintic"  # linear, cubic, or quintic
    min_angle_deg: 0.1             # Angle filter threshold
    points_per_segment: 50         # Interpolation density
```

### trajectory_executor_params.yaml
```yaml
trajectory_executor:
  ros__parameters:
    publish_rate_hz: 20.0    # Publishing rate to Pico
    start_execution: true    # Auto-start when loaded
```

## Launch Files

### 1. Offline Processing
Processes DXF file and generates complete interpolated trajectory.

```bash
ros2 launch scara_pkg_gr03 offline_processing.launch.py \
  dxf_file:=/path/to/file.dxf \
  elbow_up:=true
```

**Nodes started:**
- dxf_exporter_node_v2
- offline_trajectory_generator
- offline_inverse_kinematics
- joint_trajectory_interpolator

### 2. Trajectory Execution
Executes pre-computed trajectory.

```bash
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py \
  publish_rate_hz:=20.0 \
  start_execution:=true
```

**Nodes started:**
- trajectory_executor

### 3. Visualization Only
Visualizes robot state without executing trajectory.

```bash
ros2 launch scara_pkg_gr03 visualization_only.launch.py
```

**Nodes started:**
- scara_forward_kinematics
- rviz2

## Usage Workflow

### Complete Pipeline (Process + Execute + Visualize)

#### Terminal 1: Process DXF File
```bash
cd ~/ros2_ws_2502
source install/setup.bash
ros2 launch scara_pkg_gr03 offline_processing.launch.py \
  dxf_file:=/path/to/circle.dxf \
  elbow_up:=true
```

Wait for completion message:
```
✅ Generated XXX interpolated configurations
📤 Published interpolated trajectory
```

#### Terminal 2: Start Visualization
```bash
cd ~/ros2_ws_2502
source install/setup.bash
ros2 launch scara_pkg_gr03 visualization_only.launch.py
```

#### Terminal 3: Execute Trajectory
```bash
cd ~/ros2_ws_2502
source install/setup.bash
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py \
  publish_rate_hz:=20.0
```

### Manual Trigger (Future Enhancement)
Set `start_execution:=false` and trigger via service:
```bash
ros2 service call /start_trajectory std_srvs/srv/Trigger
```

## JSON Message Format

### joint_configurations (Raw IK Solutions)
```json
[
  {"j1": 45.5, "j2": 30.2, "j3": 10.5},
  {"j1": 46.1, "j2": 31.0, "j3": 10.5},
  ...
]
```

### interpolated_trajectory (After Interpolation)
```json
[
  {"j1": 45.5, "j2": 30.2, "j3": 10.5},
  {"j1": 45.52, "j2": 30.22, "j3": 10.5},  // Interpolated
  {"j1": 45.54, "j2": 30.24, "j3": 10.5},  // Interpolated
  ...
  {"j1": 46.1, "j2": 31.0, "j3": 10.5}
]
```

## Parameters

### DXF Exporter (scara_v2_params.yaml)
- `workspace_center_x`: X center of SCARA workspace (mm)
- `workspace_center_y`: Y center of SCARA workspace (mm)
- `scale`: Scaling factor for DXF coordinates
- `safe_height`: Z height for safe movements (mm)
- `prismatic_down_height`: Z height when drawing (mm)

### Inverse Kinematics (scara_kinematics_params.yaml)
- `elbow_up`: Use elbow-up (true) or elbow-down (false)
- `l1`, `l2`: Link lengths (mm)

### Interpolator (offline_trajectory_params.yaml)
- `interpolation_type`: "linear", "cubic", or "quintic"
- `min_angle_deg`: Minimum angle change filter (degrees)
- `points_per_segment`: Interpolation density

### Executor (trajectory_executor_params.yaml)
- `publish_rate_hz`: Publishing rate to Pico (Hz)
- `start_execution`: Auto-start execution (bool)

## Comparison: Online vs Offline

| Feature | Online System | Offline System |
|---------|---------------|----------------|
| **Computation** | Real-time during execution | Pre-computed upfront |
| **Validation** | No pre-validation | Full pre-validation |
| **Interpolation** | Waypoints only | Waypoints + Joint angles |
| **Flexibility** | Can adjust on-the-fly | Fixed after processing |
| **Smoothness** | Limited by publish rate | Optimized smoothness |
| **Complexity** | Lower | Higher |
| **Use Case** | Interactive control | Production execution |

## Troubleshooting

### No trajectory generated
- Check DXF file path is correct
- Verify `dxf_file` launch argument
- Check DXF exporter logs for parsing errors

### IK failures
- Check joint limits in logs
- Try different `elbow_up` setting
- Verify workspace center/scale parameters

### Jerky motion
- Increase `points_per_segment`
- Use quintic interpolation
- Reduce `min_angle_deg` filter

### Trajectory executes too fast/slow
- Adjust `publish_rate_hz` parameter
- Check Pico communication rate

## Examples

### Circle Drawing
```bash
# Terminal 1: Process
ros2 launch scara_pkg_gr03 offline_processing.launch.py \
  dxf_file:=~/ros2_ws_2502/src/scara_pkg_gr03/dxf/circle.dxf

# Terminal 2: Visualize
ros2 launch scara_pkg_gr03 visualization_only.launch.py

# Terminal 3: Execute
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py \
  publish_rate_hz:=20.0
```

### Complex Part with High Precision
```bash
# Use quintic interpolation with high density
ros2 launch scara_pkg_gr03 offline_processing.launch.py \
  dxf_file:=~/part.dxf

# Slow execution for precision
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py \
  publish_rate_hz:=10.0
```

## Future Enhancements

- [ ] Service-based execution trigger
- [ ] Trajectory pause/resume
- [ ] Real-time trajectory modification
- [ ] Trajectory recording/playback
- [ ] Collision detection
- [ ] Velocity profiling
- [ ] Acceleration limits

## File Locations

### Nodes
- `scara_pkg_gr03/offline_trajectory_generator.py`
- `scara_pkg_gr03/offline_inverse_kinematics.py`
- `scara_pkg_gr03/joint_trajectory_interpolator.py`
- `scara_pkg_gr03/trajectory_executor.py`

### Launch Files
- `launch/offline_processing.launch.py`
- `launch/trajectory_execution.launch.py`
- `launch/visualization_only.launch.py`

### Config Files
- `config/offline_trajectory_params.yaml`
- `config/trajectory_executor_params.yaml`
- `config/scara_v2_params.yaml` (DXF exporter)
- `config/scara_kinematics_params.yaml` (IK)
