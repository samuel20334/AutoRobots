# AutoRobots

Lab work and implementations from **Introduction to Autonomous Robots**, built on **ROS 2** and the **TurtleBot 4** platform. Each lab implements a core mobile-robotics capability from the ground up — service/action-based control, reactive navigation, probabilistic localization, and path planning — rather than relying solely on out-of-the-box ROS 2 packages.

## What's implemented

| Lab | Focus | Highlights |
|---|---|---|
| Lab 1 | ROS 2 fundamentals | Custom publisher/subscriber node, service server, and action server for TurtleBot 4 control |
| Lab 2 | Actions in depth | Arc-motion action server (`tb4_arc_action`) driving the robot along a defined arc with continuous feedback |
| Lab 3 | Reactive behaviours | `wall_follower` and `person_follower` nodes using laser-scan processing for closed-loop reactive control |
| Lab 5 | Probabilistic localization | Custom AMCL-style node with a hand-implemented **likelihood field observation model** for particle weighting |
| Lab 6 | Localization (extended) | AMCL node paired with a custom **k-d tree** for efficient nearest-neighbour particle/map queries |
| Lab 7 | Path planning | **A\*** global planner (`iar_astar_planner`) built on a custom `navfn`-style cost propagation implementation |

## Tech stack

- **ROS 2** (colcon workspaces, `rclcpp`, launch files)
- **C++** for perception, localization, and planning nodes
- **Python** for launch configuration
- **TurtleBot 4** simulation and hardware target

## Repository structure

```
AutoRobots/
├── SamLab1/  tb4_ws/src/tb4_cpp_prac1/   # pub/sub, service, and action server
├── SamLab2/  tb4_ws/src/tb4_cpp_prac2/   # arc-motion action server
├── SamLab3/  tb4_ws/src/tb4_cpp_prac3/   # wall follower / person follower
├── SamLab5/  tb4_cpp_prac5/              # AMCL + likelihood field model
├── SamLab6/  tb4_cpp_prac6/              # AMCL + k-d tree particle filter
├── SamLab7/  tb4_cpp_prac7/              # A* planner (navfn-style)
└── map/                                  # occupancy grid maps used for localization/planning labs
```

Each lab folder is a self-contained `colcon` package (or workspace) with its own `CMakeLists.txt`, `package.xml`, and `launch/` directory.

## Building & running

From inside a given lab's ROS 2 workspace (e.g. `SamLab7/tb4_cpp_prac7`):

```bash
# Build
colcon build --symlink-install
source install/setup.bash

# Launch (example: A* planner lab)
ros2 launch tb4_cpp_prac7 navigation_launch.py
```

Localization labs expect a pre-built map (see `map/`) to be supplied via the relevant `.yaml` map file at launch.

## Notes

This repository is coursework produced across a semester, so each lab folder was built and graded independently. Some interfaces evolve between labs as concepts build on one another (e.g. the k-d tree in Lab 6 extends the likelihood field model introduced in Lab 5).
