# Testbed Navigation Package

## Overview
This package provides navigation capabilities for the testbed robot using ROS2 Nav2 navigation stack. It includes AMCL localization, path planning, and autonomous navigation in simulated environments.

## Package Contents

### Launch Files
- `localization.launch.py` - Starts AMCL localization
- `map_loader.launch.py` - Loads static map for navigation
- `full_navigation.launch.py` - Complete Nav2 navigation stack

### Configuration Files
- `params/nav2_params.yaml` - Complete Nav2 navigation parameters
- `config/` - Additional configuration files

### Maps
- `maps/` - Static maps for navigation (*.pgm, *.yaml)

### RViz Configurations
- `rviz/` - RViz configuration files for navigation visualization

## Quick Start

### 1. Launch Complete System
```bash
# Terminal 1: Start robot simulation
source install/setup.bash
ros2 launch testbed_bringup testbed_full_bringup.launch.py

# Terminal 2: Start localization
source install/setup.bash
ros2 launch testbed_navigation localization.launch.py

# Terminal 3: Start navigation
source install/setup.bash
ros2 launch testbed_navigation full_navigation.launch.py
```

### 2. Set Initial Pose
1. In RViz, select **"2D Pose Estimate"** tool
2. Click and drag on the map to set robot's initial position and orientation

### 3. Send Navigation Goals
1. Select **"2D Nav Goal"** tool in RViz
2. Click and drag on the map to set destination
3. Robot will automatically plan and execute path

## Navigation Features

### ✅ What's Working
- **AMCL Localization**: Adaptive Monte Carlo Localization
- **Path Planning**: Global and local path planning
- **Obstacle Avoidance**: Dynamic obstacle detection and avoidance
- **Map Visualization**: Static map display in RViz
- **Transform Chain**: Complete TF tree (map→odom→base_footprint→base_link)

### 🔧 Configuration Highlights
- **Controller**: Regulated Pure Pursuit Controller
- **Planner**: NavFn Planner with A* algorithm
- **Costmaps**: Global and local costmaps with laser scan integration
- **Behaviors**: Spin, backup, and wait recovery behaviors

## Troubleshooting

### Map Not Visible in RViz
1. Check Fixed Frame is set to "map"
2. Verify Map display has "Transient Local" durability policy
3. Ensure map server is running: `ros2 topic list | grep map`

### Navigation Goals Not Working
1. Use "2D Nav Goal" tool (not Nav2 Goal)
2. Ensure initial pose is set with "2D Pose Estimate"
3. Check AMCL is publishing poses: `ros2 topic echo /amcl_pose`

### Robot Not Localizing
1. Set initial pose close to actual robot position
2. Move robot slightly to trigger AMCL updates
3. Check laser scan data: `ros2 topic echo /scan`

## Dependencies
- nav2_bringup
- nav2_msgs
- nav2_map_server
- nav2_amcl
- nav2_controller
- nav2_planner
- nav2_behaviors
- nav2_bt_navigator

## Parameters

### Key Navigation Parameters
```yaml
# Controller frequency
controller_frequency: 20.0

# Goal tolerances
xy_goal_tolerance: 0.25
yaw_goal_tolerance: 0.25

# Velocity limits
desired_linear_vel: 0.5
max_angular_accel: 3.2

# Robot footprint
footprint: "[ [0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15] ]"
```

### Costmap Configuration
```yaml
# Local costmap (rolling window)
width: 3
height: 3
resolution: 0.05
rolling_window: true

# Global costmap (static map)
global_frame: map
robot_base_frame: base_footprint
```

## Topics

### Subscribed Topics
- `/scan` - Laser scan data for localization and obstacle detection
- `/odom` - Odometry for robot pose estimation
- `/map` - Static map for navigation

### Published Topics
- `/amcl_pose` - Localized robot pose
- `/cmd_vel` - Velocity commands to robot
- `/local_costmap/costmap` - Local costmap for visualization
- `/global_costmap/costmap` - Global costmap for visualization

### Action Servers
- `/navigate_to_pose` - Main navigation action
- `/compute_path_to_pose` - Path planning action
- `/follow_path` - Path following action

## Performance Metrics
- **AMCL Update Rate**: ~30 Hz
- **Controller Frequency**: 20 Hz
- **Planning Response Time**: < 1 second
- **Localization Accuracy**: ±0.1m (in good conditions)

## Advanced Usage

### Custom Navigation Parameters
Edit `params/nav2_params.yaml` to customize:
- Controller behavior
- Planner algorithms
- Costmap layers
- Recovery behaviors

### Adding New Maps
1. Place map files (*.pgm, *.yaml) in `maps/` directory
2. Update map path in launch files
3. Rebuild and test navigation

### Behavior Tree Customization
Modify `default_nav_to_pose_bt_xml` parameter to use custom behavior trees for advanced navigation logic.

## Documentation
For complete setup documentation and troubleshooting guide, see:
`/NAVIGATION_SETUP_DOCUMENTATION.md` in the workspace root.

---
**Package Version**: 1.0  
**ROS2 Version**: Humble  
**Last Updated**: August 7, 2025
