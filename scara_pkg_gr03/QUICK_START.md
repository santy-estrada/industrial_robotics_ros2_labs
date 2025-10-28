# Offline Trajectory System - Quick Start Guide

## 🚀 Quick Start

### One-Line Complete Test
```bash
# Terminal 1: Process trajectory (wait for completion)
ros2 launch scara_pkg_gr03 offline_processing.launch.py dxf_file:=~/ros2_ws_2502/src/scara_pkg_gr03/dxf/circle.dxf

# Terminal 2: Visualize
ros2 launch scara_pkg_gr03 visualization_only.launch.py

# Terminal 3: Execute
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py
```

## 📊 System Status Check

### Check if trajectory is ready
```bash
ros2 topic echo /interpolated_trajectory --once
```

### Monitor execution progress
```bash
ros2 topic echo /inv_kin
```

### View all active topics
```bash
ros2 topic list
```

## ⚙️ Common Parameter Adjustments

### Faster/Slower Execution
```bash
# Faster (60 Hz)
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py publish_rate_hz:=60.0

# Slower (10 Hz)
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py publish_rate_hz:=10.0
```

### Change Interpolation Type
Edit `config/offline_trajectory_params.yaml`:
```yaml
interpolation_type: "linear"   # Fast, simple
interpolation_type: "cubic"    # Smooth, moderate
interpolation_type: "quintic"  # Very smooth, complex
```

### Adjust Trajectory Density
Edit `config/offline_trajectory_params.yaml`:
```yaml
points_per_segment: 20   # Fewer points, faster
points_per_segment: 50   # Default
points_per_segment: 100  # More points, smoother
```

## 🐛 Debugging Commands

### Check node status
```bash
ros2 node list
```

### View node info
```bash
ros2 node info /offline_inverse_kinematics
ros2 node info /joint_trajectory_interpolator
ros2 node info /trajectory_executor
```

### Check parameter values
```bash
ros2 param list /joint_trajectory_interpolator
ros2 param get /joint_trajectory_interpolator interpolation_type
```

### Monitor topic rate
```bash
ros2 topic hz /inv_kin
```

## 📁 File Structure
```
scara_pkg_gr03/
├── scara_pkg_gr03/
│   ├── offline_trajectory_generator.py
│   ├── offline_inverse_kinematics.py
│   ├── joint_trajectory_interpolator.py
│   └── trajectory_executor.py
├── launch/
│   ├── offline_processing.launch.py
│   ├── trajectory_execution.launch.py
│   └── visualization_only.launch.py
├── config/
│   ├── offline_trajectory_params.yaml
│   └── trajectory_executor_params.yaml
└── OFFLINE_TRAJECTORY_SYSTEM.md (full documentation)
```

## 🔄 Workflow Diagram
```
   ┌──────────────┐
   │  DXF File    │
   └──────┬───────┘
          │
          ▼
   ┌──────────────────────┐
   │  DXF Exporter V2     │ ← scara_v2_params.yaml
   └──────┬───────────────┘
          │ /dxf_pointcloud
          ▼
   ┌──────────────────────────┐
   │  Offline Traj Generator  │
   └──────┬───────────────────┘
          │ /trajectory_waypoints
          ▼
   ┌──────────────────────────┐
   │  Offline Inverse Kin     │ ← scara_kinematics_params.yaml
   └──────┬───────────────────┘
          │ /joint_configurations
          ▼
   ┌──────────────────────────┐
   │  Joint Interpolator      │ ← offline_trajectory_params.yaml
   └──────┬───────────────────┘
          │ /interpolated_trajectory
          ▼
   ┌──────────────────────────┐
   │  Trajectory Executor     │ ← trajectory_executor_params.yaml
   └──────┬───────────────────┘
          │ /inv_kin (Twist)
          ▼
   ┌──────────────┐
   │   Pico       │
   └──────────────┘
```

## 💡 Tips

1. **Always wait** for offline processing to complete before starting execution
2. **Check logs** for IK failures during processing
3. **Use visualization** to verify trajectory before executing
4. **Start slow** (10-20 Hz) when testing new parts
5. **Save successful** configurations for later replay
6. **Monitor /inv_kin** topic during execution to verify communication

## ⚠️ Common Issues

| Problem | Solution |
|---------|----------|
| "No trajectory loaded" | Run offline_processing.launch.py first |
| Jerky motion | Increase points_per_segment or use quintic |
| Too slow | Increase publish_rate_hz |
| IK failures | Check workspace center/scale, try elbow_up:=false |
| Nodes not starting | Check QoS settings, rebuild package |

## 📝 Example Usage Patterns

### Testing New DXF File
```bash
# 1. Process with low density first
ros2 launch scara_pkg_gr03 offline_processing.launch.py \
  dxf_file:=~/new_part.dxf

# 2. Visualize only
ros2 launch scara_pkg_gr03 visualization_only.launch.py

# 3. If looks good, execute slowly
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py \
  publish_rate_hz:=10.0
```

### Production Run
```bash
# 1. Process with high quality
# Edit config/offline_trajectory_params.yaml:
#   interpolation_type: "quintic"
#   points_per_segment: 100

# 2. Process
ros2 launch scara_pkg_gr03 offline_processing.launch.py \
  dxf_file:=~/production_part.dxf

# 3. Execute at optimal rate
ros2 launch scara_pkg_gr03 trajectory_execution.launch.py \
  publish_rate_hz:=60.0
```
