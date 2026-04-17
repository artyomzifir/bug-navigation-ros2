# Path Planning Homework for ROS 2

This repository contains a ROS 2 Humble homework project on mobile robot path planning in Gazebo. It includes three planners built on top of the differential drive robot from `rtabmap_diff_drive_tutorial`: Bug0, Bug1, and a global A* planner with a separate path follower. Bug0 and Bug1 are reactive planners based on `LaserScan` and odometry, while A* plans on an occupancy grid generated from RTAB-Map.

The project is intended for comparing reactive and map-based navigation. Bug planners can run with only simulation and localization, while A* additionally requires a valid `/map` and TF in the `map` frame.

## Quick start

```bash
docker compose up
docker compose exec planning bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
````

Run simulation:

```bash
ros2 launch rtabmap_diff_drive_tutorial robot_simulation.launch.py
```

Run localization:

```bash
ros2 launch rtabmap_diff_drive_tutorial rtabmap_no_mapping.launch.py database_path:=/root/.ros/rtabmap.db
```

Run Bug0:

```bash
ros2 launch bug_navigation bug0.launch.py
```

Run Bug1:

```bash
ros2 launch bug_navigation bug1.launch.py
```

Run A* with path follower:

```bash
ros2 launch a_star_planner a_star.launch.py
```

Use RViz to send a goal with `2D Goal Pose`.
