# Path Planning Homework for ROS 2

![A* demo](demo-a-star.gif)

This repository contains a ROS 2 Humble homework project on mobile robot path planning in Gazebo. It includes three planners built on top of a lidar-equipped differential drive robot from `robot_bringup`: Bug0, Bug1, and a global A* planner with a separate path follower. Bug0 and Bug1 are reactive planners based on `LaserScan` and odometry, while A* plans on an occupancy grid built live from the lidar scan.

The project is intended for comparing reactive and map-based navigation. Bug planners only need the simulation running, while A* additionally requires a `/map` — provided automatically by the live mapper included in `a_star.launch.py`.

## Quick start

```bash
docker compose up
docker compose exec planning bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run simulation:

```bash
ros2 launch robot_bringup simulation.launch.py
```

Run Bug0:

```bash
ros2 launch bug_navigation bug0.launch.py
```

Run Bug1:

```bash
ros2 launch bug_navigation bug1.launch.py
```

Run A* with live mapper and path follower:

```bash
ros2 launch a_star_planner a_star.launch.py
```

Use RViz to send a goal with `2D Goal Pose`.
