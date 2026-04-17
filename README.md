# Path planning homework (Bug 0, Bug 1, A*)

This workspace extends the `rtabmap_diff_drive_tutorial` lab. It adds three
planners on top of the existing differential-drive robot in Gazebo:

| Package          | Node                  | Algorithm                                       |
|------------------|-----------------------|-------------------------------------------------|
| `bug_navigation` | `bug_planner_node`    | Tangent Bug / Bug 2 (from the tutorial; TODO filled in) |
| `bug_navigation` | `bug0_planner_node`   | **Task 1 — Bug 0** (reactive, leaves wall as soon as goal direction is clear) |
| `bug_navigation` | `bug1_planner_node`   | **Task 2 — Bug 1** (full circumnavigation + closest-point) |
| `a_star_planner` | `a_star_planner_node` | **Task 3 — A*** global planner on `nav_msgs/OccupancyGrid` |
| `a_star_planner` | `path_follower_node`  | Carrot-follower for the A* path                 |

The three Bug nodes are reactive — they only need `/odom`, `/scan`, and a
`/goal_pose`. A* additionally needs a published map on `/map` (from
`nav2_map_server` or RTAB-Map).

---

## 1. Workspace layout

Mount this directory as `/ros2_ws/src` inside your container:

```
ros2_ws/src/
├── README.md                      <- this file
├── rtabmap_diff_drive_tutorial/   <- unchanged from the previous lab
├── bug_navigation/
│   ├── bug_navigation/
│   │   ├── bug_planner.py         <- Tangent Bug (TODO filled in)
│   │   ├── bug0_planner.py        <- NEW (Task 1)
│   │   └── bug1_planner.py        <- NEW (Task 2)
│   ├── launch/{bug0,bug1}.launch.py
│   ├── package.xml
│   └── setup.py
└── a_star_planner/                <- NEW (Task 3)
    ├── a_star_planner/
    │   ├── a_star.py              <- pure algorithm, no ROS deps
    │   ├── a_star_planner_node.py
    │   └── path_follower_node.py
    ├── launch/a_star.launch.py
    ├── package.xml
    └── setup.py
```

---

## 2. Docker setup

Any image with **ROS 2 Humble + Gazebo Classic 11 + RTAB-Map** works. If you
already have one from the previous lab, use it. Otherwise, the canonical
image is `osrf/ros:humble-desktop-full`; install RTAB-Map and Nav2 map_server
on top of it.

### Minimal Dockerfile

```dockerfile
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
      ros-humble-rtabmap-ros \
      ros-humble-nav2-map-server \
      ros-humble-nav2-lifecycle-manager \
      ros-humble-teleop-twist-keyboard \
      ros-humble-tf2-geometry-msgs \
      ros-humble-gazebo-ros-pkgs \
      python3-colcon-common-extensions \
      python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Convenient sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc
```

### Run the container with GUI passthrough (Linux host)

```bash
xhost +local:root  # allow the container to talk to your X server

docker run -it --rm \
    --name planning \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD":/ros2_ws/src \
    -v "$HOME/.ros":/root/.ros \
    planning-image bash
```

The `-v "$HOME/.ros":/root/.ros` mount keeps your `rtabmap.db` between runs.

### Build

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to Python files take effect without a
rebuild — handy when iterating on the planners.

For extra terminals into the same container:

```bash
docker exec -it planning bash
# inside, every new shell needs:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

---

## 3. Verifying Tasks 1 and 2 (Bug 0 / Bug 1)

These two only need the simulation, localization, and a goal in RViz.
The `/map` is **not** required for them.

### Terminal A — simulation

```bash
ros2 launch rtabmap_diff_drive_tutorial robot_simulation.launch.py
```

Wait until Gazebo is up and you can see the robot in `test.world`.

### Terminal B — localization (uses your saved DB from the previous lab)

```bash
ros2 launch rtabmap_diff_drive_tutorial rtabmap_no_mapping.launch.py \
    database_path:=/root/.ros/rtabmap.db
```

This brings up RTAB-Map in localization-only mode + RViz. Confirm in RViz
that the robot is correctly localized inside the saved map (TF
`map -> odom -> base_link` is being published).

> **No saved DB?** You can still test the Bug planners — they don't depend
> on `/map`. Just skip RTAB-Map; the robot will navigate in the `odom`
> frame using LIDAR only, and goals you set in RViz must be in the `odom`
> frame (in RViz, change the "Fixed Frame" to `odom` before clicking
> "2D Goal Pose").

### Terminal C — pick a planner

```bash
# Task 1
ros2 launch bug_navigation bug0.launch.py

# Task 2
ros2 launch bug_navigation bug1.launch.py

# Reference Tangent Bug (from the previous lab)
ros2 run bug_navigation bug_planner_node
```

### Sending a goal

In the RViz window, click the **2D Goal Pose** button on the toolbar, then
click on the map. The PoseStamped is published on `/goal_pose` (frame
`map`); the Bug nodes transform it into `odom` automatically.

### What to look for

In RViz, add a `Path` display and subscribe to:

* `/bug0_path` for Bug 0
* `/bug1_path` for Bug 1
* `/robot_path` for Tangent Bug

This shows the actual trajectory the robot has taken since the goal was
sent — useful for the comparisons asked for in the homework.

**Things to test**, per the task spec:

* A goal behind a **single convex obstacle** — all three planners reach it.
  Bug 0 ≈ Tangent Bug ≪ Bug 1 in path length.
* A goal that requires entering a **U-shaped pocket** with the goal on the
  far side — Bug 0 will typically loop or get stuck (it sees a "clear"
  heading toward the goal from inside the pocket and abandons the wall too
  early). Bug 1 always escapes by walking the full perimeter and then
  driving to the closest boundary point. This is exactly the classic
  Bug 0 trap and is the "observation on entrapment scenarios" the
  assignment asks for.

### Tuning

All three nodes share these parameters:

```bash
ros2 run bug_navigation bug0_planner_node --ros-args \
    -p obstacle_threshold:=0.5 \
    -p wall_follow_dist:=0.4 \
    -p linear_speed:=0.2 \
    -p angular_speed:=0.5
```

Bug 1 has two extra knobs:

* `hit_point_tolerance` (default 0.3 m) — how close the robot has to come
  back to the original hit point to count as "lap done".
* `min_circumnav_distance` (default 1.0 m) — guards against the lap
  trigger firing instantly on the first scan tick.

Bug 0 has one extra knob:

* `goal_clear_arc_deg` (default 5°) — half-width of the cone in the goal
  direction that must be clear before leaving the wall.

---

## 4. Verifying Task 3 (A*)

A* needs an `OccupancyGrid` on `/map`. There are two common ways to get one:

### Option A — export your RTAB-Map to a `.pgm` and use `nav2_map_server` (recommended)

In a one-off terminal, with RTAB-Map running, dump the grid to PGM/YAML:

```bash
mkdir -p /ros2_ws/maps
ros2 run rtabmap_ros map_assembler --ros-args -p grid_global_map_topic:=/map  # may not be needed
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/test_map
```

You'll get `/ros2_ws/maps/test_map.pgm` and `test_map.yaml`. Then you can
run the simulation and the map server independently of RTAB-Map.

Run the four terminals:

```bash
# A — simulation
ros2 launch rtabmap_diff_drive_tutorial robot_simulation.launch.py

# B — map server (publishes /map with TRANSIENT_LOCAL durability)
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=/ros2_ws/maps/test_map.yaml \
    -p use_sim_time:=true
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p use_sim_time:=true \
    -p autostart:=true \
    -p node_names:="['map_server']"

# C — localization, so map -> odom TF exists
ros2 launch rtabmap_diff_drive_tutorial rtabmap_no_mapping.launch.py \
    database_path:=/root/.ros/rtabmap.db

# D — A* planner + path follower
ros2 launch a_star_planner a_star.launch.py
```

### Option B — let RTAB-Map publish `/map` directly

`rtabmap_no_mapping.launch.py` already publishes the grid on `/map`. Just
skip terminal B from the list above. The downside is that RTAB-Map's grid
QoS sometimes doesn't match what the A* node expects; if `/map` arrives but
the planner says "no map yet", check `ros2 topic info /map -v`.

### Sending a goal

Same workflow as before — **2D Goal Pose** in RViz. The A* node logs:

```
Planning (1.20, 0.30) (cell (124, 144)) -> (4.50, -2.10) (cell (82, 210))
Published path with 47 waypoints.
```

Add a `Path` display in RViz subscribed to `/planned_path` to see the plan.
The path follower will start driving immediately.

If the path goes through walls, increase `robot_radius_m`:

```bash
ros2 run a_star_planner a_star_planner_node --ros-args -p robot_radius_m:=0.25
```

If A* says "no path found" near the goal, your goal might be sitting on
top of an inflated obstacle. Pick a goal slightly further into open space,
or lower the radius.

---

## 5. Implementation notes

### Bug 0 vs Bug 1 vs Tangent Bug — leave conditions side by side

| Algorithm   | Leave wall when…                                                                      |
|-------------|---------------------------------------------------------------------------------------|
| Bug 0       | Direct heading toward the goal is clear in the laser scan                             |
| Bug 1       | A full lap of the obstacle has been completed; then walk to the closest boundary point |
| Tangent Bug | Goal heading is clear AND robot is closer to the goal than at the hit point AND on the M-line |

Bug 0 has the simplest rule and the worst worst-case (entrapment). Bug 1 is
complete but slow. Tangent Bug is usually the best of both worlds in
practice.

### A* heuristic

The grid is 8-connected. Manhattan distance overestimates the cost of
diagonal moves and breaks optimality, so we use the **octile** distance:

```
h = (dr + dc) + (sqrt(2) - 2) * min(dr, dc)
```

This is admissible and consistent for unit-cost 4-connected moves and
sqrt(2)-cost diagonal moves, so A* returns an optimal path.

### Obstacle inflation

Before running A*, the occupancy grid is dilated by
`ceil(robot_radius / resolution)` cells using a Euclidean disk kernel. Without
this, A* will happily route the centre of the robot through cells right
next to a wall, which means the robot actually clips the wall.

### Frames

* Bug nodes work in `odom`. They TF-transform the incoming goal (which RViz
  publishes in `map`) into `odom` once on receipt. This way odom drift
  doesn't make the goal "move" mid-run.
* A* and the path follower work in `map`. They look up
  `map -> base_link` from TF every cycle, so they automatically benefit
  from RTAB-Map's pose corrections.

### Topic recap

| Topic            | Type                      | Direction              |
|------------------|---------------------------|------------------------|
| `/odom`          | `nav_msgs/Odometry`       | Bug nodes subscribe    |
| `/scan`          | `sensor_msgs/LaserScan`   | Bug nodes subscribe    |
| `/goal_pose`     | `geometry_msgs/PoseStamped` | All planners subscribe |
| `/cmd_vel`       | `geometry_msgs/Twist`     | All planners publish   |
| `/map`           | `nav_msgs/OccupancyGrid`  | A* subscribes (TRANSIENT_LOCAL) |
| `/planned_path`  | `nav_msgs/Path`           | A* publishes; follower subscribes |
| `/bug0_path`     | `nav_msgs/Path`           | Bug 0 publishes (visualization) |
| `/bug1_path`     | `nav_msgs/Path`           | Bug 1 publishes (visualization) |
| `/robot_path`    | `nav_msgs/Path`           | Tangent Bug publishes (visualization) |

---

## 6. Troubleshooting

* **"Waiting for scan / odom..." forever** — check `ros2 topic list` and
  `ros2 topic hz /scan /odom`. The Gazebo plugin in `robot.urdf.xacro`
  needs to be loaded; if you don't see `/scan`, look for the
  `gpu_lidar`/`hokuyo` plugin block.
* **A* never receives the map** — `/map` from `nav2_map_server` is
  TRANSIENT_LOCAL durability. The planner's QoS already matches; if you
  publish from a custom node, set `durability=TRANSIENT_LOCAL` on the
  publisher or the subscription will silently drop the message.
* **Robot drives into the wall when following A*** — increase
  `robot_radius_m` (default 0.18 m). The default fits the URDF in this
  tutorial; if you changed the chassis, recompute it.
* **Bug 0 oscillates in front of an open doorway** — `goal_clear_arc_deg`
  is too narrow; increase to ~10°. Or `obstacle_threshold` is too tight.
* **Bug 1 finishes a lap immediately and gives up** — `min_circumnav_distance`
  is too small for the obstacle scale; bump it to 2 × the perimeter
  diameter you expect.

Good luck.
