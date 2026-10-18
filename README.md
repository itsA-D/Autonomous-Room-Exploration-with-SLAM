# Autonomous Room Mapping

ROS 2 package for autonomous exploration of a simulated TurtleBot3 room while
`slam_toolbox` builds a 2D occupancy map. The project uses a lightweight,
reactive controller rather than a navigation planner: the explorer reads
LiDAR scans, drives forward when space is available, and turns toward the side
with more measured space when an obstacle is close.

## Project Overview

The project addresses a basic robotics mapping workflow:

1. Gazebo starts a TurtleBot3 simulation.
2. The robot publishes LiDAR scans and odometry.
3. The `autonomous_exploration_node` reacts to LiDAR measurements and publishes
   velocity commands.
4. `slam_toolbox` consumes the robot's scan and odometry data to build an
   occupancy grid.
5. RViz2 displays the resulting map and simulation data.

The explorer does not select navigation goals, plan paths, or perform frontier
exploration. Its movement strategy is intentionally simple and reactive.

## Features

- Autonomous forward motion in a simulated room.
- LiDAR-based nearest-obstacle detection.
- Reactive obstacle avoidance using left/right scan-sector averages.
- Randomized steering while moving through open space.
- Online 2D SLAM with `slam_toolbox`.
- RViz2 visualization of the simulated robot and occupancy map.
- ROS 2 launch file that starts Gazebo, SLAM, RViz2, and the explorer node.

## Tech Stack

The current implementation uses:

- ROS 2 Python (`rclpy`)
- Python 3
- NumPy
- Gazebo through `turtlebot3_gazebo`
- TurtleBot3 simulation packages
- `slam_toolbox`
- RViz2
- ROS 2 messages:
  - `sensor_msgs/msg/LaserScan`
  - `geometry_msgs/msg/Twist`
  - `nav_msgs/msg/OccupancyGrid`
- `colcon` for building
- `rosdep` for dependency installation
- `pytest`, `ament_flake8`, `ament_pep257`, and `ament_copyright` for the
  configured test checks

## System Architecture

```text
Gazebo / TurtleBot3
    │
    ├── /scan ───────────────► autonomous_exploration_node
    │                              │
    │                              └── /cmd_vel ─────► Gazebo / TurtleBot3
    │
    ├── scan + odometry ─────► slam_toolbox
    │                              │
    │                              └── /map ─────────► RViz2
    │                                               │
    └────────────────────── /map ──────────────────┘
```

### ROS Nodes

- **Gazebo / TurtleBot3**: Simulates the robot and environment. The
  `turtlebot3_world.launch.py` launch file is included from the installed
  `turtlebot3_gazebo` package.
- **`room_mapping_explorer`**: The node implemented by
  `the_mapper/autonomous_exploration_node.py`. It evaluates LiDAR ranges and
  publishes velocity commands.
- **`slam_toolbox`**: Runs `async_slam_toolbox_node` with the installed
  `mapper_params_online_async.yaml` configuration.
- **RViz2**: Runs with the TurtleBot3 Gazebo RViz configuration and visualizes
  the map.

### ROS Topics

| Topic | Message type | Role |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR input used for obstacle detection and steering decisions |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy grid published by SLAM; stored by the explorer and displayed by RViz2 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands published by the explorer to move the robot |

The explorer subscribes to `/map` and stores the latest occupancy grid, but the
current movement controller does not use map contents to choose goals or plan
paths.

## Project Structure

```text
.
├── the_mapper/
│   ├── __init__.py
│   └── autonomous_exploration_node.py
├── bringup/
│   └── room_mapping_launch.py
├── resource/
│   └── the_mapper
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
├── setup.cfg
└── .gitignore
```

`bringup/` is the source directory for launch files. In
[setup.py](setup.py), those files are collected from `bringup/*.py` and
installed into the standard ROS 2 package destination
`share/the_mapper/launch`.

## Prerequisites and Installation

Use a ROS 2 environment with the following packages available:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `slam_toolbox`
- `turtlebot3_gazebo`
- RViz2 and the ROS 2 launch packages
- Python 3 with NumPy
- `colcon` and `rosdep`

The repository does not state a single tested ROS 2 distribution. Use a
distribution whose package repositories provide the dependencies above, and
source that distribution before building.

Create or use a ROS 2 workspace and place this package under `src/`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Copy or clone this package here.
```

Install dependencies from the workspace root:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Set the TurtleBot3 model used by the launch workflow:

```bash
export TURTLEBOT3_MODEL=burger
```

## Build and Run

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select the_mapper
```

Source the workspace:

```bash
source install/setup.bash
```

Launch Gazebo, SLAM, RViz2, and the explorer:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch the_mapper room_mapping_launch.py
```

To run only the installed explorer executable while Gazebo and its required
topics are already running:

```bash
ros2 run the_mapper autonomous_exploration_node
```

## Configuration

### Launch argument

The launch file declares one argument:

| Argument | Default | Description |
|---|---:|---|
| `use_sim_time` | `true` | Uses the Gazebo simulation clock for SLAM, RViz2, and the explorer node |

Example:

```bash
ros2 launch the_mapper room_mapping_launch.py use_sim_time:=true
```

### Explorer controller values

These are class attributes in
`the_mapper/autonomous_exploration_node.py`, not ROS parameters:

| Value | Default | Purpose |
|---|---:|---|
| `forward_speed` | `0.22` m/s | Forward speed when no obstacle is within the threshold |
| `turn_speed` | `0.4` rad/s | Angular speed during obstacle avoidance |
| `obstacle_distance_threshold` | `0.5` m | Nearest LiDAR distance that activates avoidance |
| Control-loop interval | `0.1` s | Frequency of velocity-command decisions |
| Direction-change probability | `0.05` per control cycle | Probability of flipping the random steering direction in open space |

When avoiding an obstacle, the current implementation sets a forward velocity
of `0.18` m/s and turns toward the scan sector with the larger average range.
Infinite LiDAR readings are replaced with the sensor's `range_max`.

## Testing and Troubleshooting

The repository contains ROS 2 linter wrapper tests:

- `test/test_flake8.py` checks Python style with `ament_flake8`.
- `test/test_pep257.py` checks docstring conventions with `ament_pep257`.
- `test/test_copyright.py` is currently skipped because no project copyright
  header is configured.

These are lint and repository-quality checks, not functional tests of obstacle
avoidance, SLAM accuracy, or robot motion. No unit or integration tests for the
exploration algorithm are currently included.

Run the configured checks after building:

```bash
colcon test --packages-select the_mapper
colcon test-result --verbose
```

Useful inspection commands:

```bash
ros2 pkg executables the_mapper
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /map
ros2 topic echo /cmd_vel
ros2 node list
```

If the launch command fails:

1. Confirm the ROS 2 environment is sourced.
2. Confirm `TURTLEBOT3_MODEL=burger` is exported.
3. Run `rosdep install --from-paths src --ignore-src -r -y` again.
4. Rebuild and source the workspace:
   `colcon build --packages-select the_mapper` followed by
   `source install/setup.bash`.
5. Confirm that `turtlebot3_gazebo`, `slam_toolbox`, and RViz2 are installed.

If the robot does not move, check that `/scan` is publishing and that the
explorer is publishing `geometry_msgs/msg/Twist` messages on `/cmd_vel`.

## Limitations and Future Improvements

Current limitations:

- Exploration is reactive and does not use frontier detection.
- There is no goal selection, global path planning, Nav2 integration, or
  recovery behavior.
- The occupancy grid is stored by the explorer but is not used for navigation
  decisions.
- Steering is randomized in open space and is not deterministic.
- The controller assumes LiDAR scans contain usable range data.
- There are no automated functional tests for motion behavior or map quality.

Potential improvements include:

- Add frontier detection and map-based exploration goals.
- Integrate Nav2 for planning, localization, and recovery behaviors.
- Expose motion values as declared ROS parameters.
- Add safety handling for empty or invalid LiDAR arrays.
- Add unit tests for scan processing and command generation, plus simulation
  integration tests.

## License

The package metadata declares the MIT license. See
[package.xml](package.xml) and [setup.py](setup.py).
