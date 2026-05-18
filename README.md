# Mechatronics

ROS 2 Humble workspace for a custom 4-wheeled mecanum-drive robot equipped with an Intel RealSense D435 depth camera. The stack covers URDF description, Gazebo simulation, RTAB-Map SLAM, map export, and Nav2 autonomous navigation.

This can serve as a reference for integrating SLAM and navigation on any custom robot (not just TurtleBot or Clearpath platforms).

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-desktop`)
- Gazebo Classic (ships with `ros-humble-desktop`)
- Nav2: `sudo apt install ros-humble-nav2-bringup`

## Repository Structure

```
Mechatronics/
├── bot_ws/                          # primary ROS 2 workspace
│   └── src/
│       ├── bot_urdf/                # robot description, launches, maps
│       │   ├── urdf/
│       │   │   └── bot.urdf.xacro   # robot model (xacro)
│       │   ├── meshes/              # STL meshes from SolidWorks
│       │   ├── config/              # nav2_params.yaml, mapping.rviz, nav.rviz
│       │   ├── launch/              # all launch files
│       │   ├── worlds/              # Gazebo world files
│       │   └── maps/                # rtabmap.db, kitchen.pgm, kitchen.yaml
│       └── realsense-ros/           # cloned third-party (D435 description)
├── rtabmap_ws/                      # RTAB-Map workspace (built separately)
│   └── src/
│       ├── rtabmap/                 # core RTAB-Map C++ library
│       └── rtabmap_ros/             # ROS 2 wrapper
└── common_errors.md                 # troubleshooting reference
```

## 1. URDF: From CAD to ROS

The robot was designed in SolidWorks and exported using the [SolidWorks to URDF Exporter](https://wiki.ros.org/sw_urdf_exporter). This produces a URDF file along with STL meshes for each link.

The exported URDF was then converted to xacro (`bot.urdf.xacro`) to support including the RealSense D435 camera macro. A joint names config file is required at `config/joint_names_bot.yaml`:

```yaml
controller_joint_names: ['', 'base_to_whl_1', 'base_to_whl_2', 'base_to_whl_3', 'base_to_whl_4', ]
```

**Visualize the URDF in RViz:**

```bash
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf display.launch.py
```

This launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz. Use the GUI sliders to test joint articulation.

> If RViz shows "Package does not exist" errors for meshes, check [common_errors.md [E3]](common_errors.md).

## 2. Gazebo Simulation

```bash
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf gazebo.launch.py
```

This spawns the robot in an empty Gazebo world with:
- **`libgazebo_ros_planar_move.so`**: holonomic base motion (see below)
- **`libgazebo_ros_joint_state_publisher.so`**: publishes `/joint_states`
- A simulated RealSense D435 depth camera (see section 3)

> If the robot spawns but is invisible (no meshes), check [common_errors.md [E1]](common_errors.md).

### Drive model in simulation

The bot is designed as a **mecanum** platform, but individual wheel rollers, friction, or motor torques are deliberately **not** simulated for the sake of this project. Instead, `libgazebo_ros_planar_move.so` applies **`geometry_msgs/Twist`** commands (`linear.x`, `linear.y`, `angular.z`) directly to the base. That gives **holonomic, mecanum-like motion** (forward, strafe, rotate) without low-level wheel control.

On real hardware, this plugin is replaced by a motor driver (e.g. `ros2_control` with mecanum inverse kinematics) that still consumes `/cmd_vel`.

**Teleoperation:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can add Gazebo models (e.g. the Cafe) to the empty world for a more interesting environment.

**Key topics published:**

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop or Nav2 |
| `/odom` | `nav_msgs/Odometry` | planar move plugin |
| `/joint_states` | `sensor_msgs/JointState` | joint state plugin |

## 3. RealSense D435 Camera

The dummy camera mesh from the original SolidWorks export was replaced with an Intel RealSense D435 using the `realsense2_description` xacro macro. This provides accurate D435 geometry, proper optical frames (`camera_depth_optical_frame`, `camera_color_optical_frame`, etc.), and the D435 DAE mesh.

### 3.1 Dependencies

```bash
# Clone into bot_ws/src (already done):
cd ~/Mechatronics/bot_ws/src
git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master
# Build only the description package (no SDK required):
colcon build --packages-up-to realsense2_description
```

For the real hardware driver, also install:
```bash
sudo apt install ros-humble-realsense2-camera
```

### 3.2 Simulation

The Gazebo depth camera sensor is defined in `bot.urdf.xacro` using `libgazebo_ros_camera.so` with sensor type `depth`, attached to `camera_link`. This is important: attaching to an empty frame like `camera_depth_frame` causes the sensor to be silently dropped due to Gazebo's fixed-joint link merging (see [common_errors.md [E2]](common_errors.md)).

> If Gazebo spawns the robot but no camera topics appear, check [common_errors.md [E2]](common_errors.md).

**Camera topics published in simulation:**

| Topic | Type | Content |
|---|---|---|
| `/camera/camera/image_raw` | `sensor_msgs/Image` | RGB image (640x480) |
| `/camera/camera/depth/image_raw` | `sensor_msgs/Image` | Depth image (640x480) |
| `/camera/camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics |
| `/camera/camera/points` | `sensor_msgs/PointCloud2` | 3D point cloud |

### 3.3 Real Hardware

```bash
ros2 launch bot_urdf realsense.launch.py
```

This starts the `realsense2_camera` driver node with color at 640x480@30fps, depth at 640x480@15fps, and pointcloud enabled. The topic names match the simulation topics, so downstream nodes (RTAB-Map, Nav2) work without changes.

## 4. RTAB-Map SLAM

[RTAB-Map](https://introlab.github.io/rtabmap/) provides RGB-D SLAM with loop closure detection, visual odometry, and occupancy grid generation.

### 4.1 Installation

RTAB-Map is built in a **separate workspace** (`rtabmap_ws`) so that iterating on `bot_ws` doesn't trigger long rebuilds.

```bash
# System dependencies
sudo apt install ros-humble-pcl-ros ros-humble-libg2o ros-humble-grid-map-ros ros-humble-imu-filter-madgwick

# Clone
mkdir -p ~/Mechatronics/rtabmap_ws/src && cd ~/Mechatronics/rtabmap_ws/src
git clone https://github.com/introlab/rtabmap.git
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git

# Build (takes ~8 minutes)
cd ~/Mechatronics/rtabmap_ws
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j12"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4.2 Running SLAM

Requires **two terminals** (Gazebo already running in terminal 1):

```bash
# Terminal 1: Gazebo
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf gazebo.launch.py

# Terminal 2: RTAB-Map
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf mapping.launch.py use_sim_time:=true
```

> Terminal 2 must source `rtabmap_ws` before `bot_ws` (see [common_errors.md [E5]](common_errors.md)).

`mapping.launch.py` starts two nodes:
- **`rgbd_odometry`**: visual odometry from RGB+depth, publishes `/odom`
- **`rtabmap`**: SLAM, publishes `/map`, `/cloud_map`, and TF

Both use `subscribe_depth: True` and `approx_sync: True`. Do not use `subscribe_rgbd: True`; our camera publishes separate RGB and depth topics, not a combined `/rgbd_image`.

> If RTAB-Map logs "Did not receive data" or RViz shows no map, check [common_errors.md [E4]](common_errors.md).

For **real hardware**:
```bash
ros2 launch bot_urdf mapping.launch.py use_sim_time:=false
```

### 4.3 Visualizing the Map

```bash
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
rviz2 -d ~/Mechatronics/bot_ws/src/bot_urdf/config/mapping.rviz
```

Set **Fixed Frame** to `map`. Drive the robot with `teleop_twist_keyboard` to build the map.

> Source both workspaces before RViz, or mesh paths fail (see [common_errors.md [E3]](common_errors.md)).

**Key RTAB-Map topics:**

| Topic | Type | Content |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 2D grid map |
| `/odom` | `nav_msgs/Odometry` | Visual odometry |
| `/cloud_map` | `sensor_msgs/PointCloud2` | 3D map |

### 4.4 Saving the Map

RTAB-Map saves its database to `~/.ros/rtabmap.db` by default. Copy it to `maps/` if you want it in the package:

```bash
cp ~/.ros/rtabmap.db ~/Mechatronics/bot_ws/src/bot_urdf/maps/rtabmap.db
```

**Export a 2D occupancy grid (.pgm + .yaml) for Nav2:**

```bash
rtabmap-databaseViewer /path/to/rtabmap.db
# Edit > Regenerate local grids, then File > Export 2D Grid Map (.pgm)
```

Example maps in the repo: `maps/kitchen.pgm`, `maps/kitchen.yaml`, `maps/rtabmap.db`.

## 5. Nav2 Autonomous Navigation

Nav2 plans paths on the static map and drives the robot to goals set in RViz. **AMCL is not used.** RTAB-Map runs in **localization mode** instead: it loads `rtabmap.db`, publishes `map` → `odom` TF, and replaces laser-based localization.

Obstacle avoidance uses the **depth camera**, not lidar. Depth images are converted to point clouds and segmented into ground/obstacles by `rtabmap_util`, which feed Nav2's local costmap.

### 5.1 Architecture

| Component | Role |
|---|---|
| `map_server` | Loads `kitchen.yaml` / `kitchen.pgm` |
| `rgbd_odometry` | Visual odometry (TF disabled in sim; Gazebo publishes `odom` → `base_link`) |
| `rtabmap` (localization) | `map` → `odom` TF, replaces AMCL |
| `point_cloud_xyz` + `obstacles_detection` | `/camera/obstacles`, `/camera/ground` for local costmap |
| Nav2 stack | Planner (Navfn), controller (DWB), behavior tree |

Configuration: `config/nav2_params.yaml`. RViz layout: `config/nav.rviz`.

### 5.2 Running Navigation (simulation)

Use **three terminals**. Load the same Gazebo world you mapped (e.g. add the kitchen model manually if you built the map there).

```bash
# Terminal 1: Gazebo
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf gazebo.launch.py

# Terminal 2: Navigation
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf navigation.launch.py use_sim_time:=true

# Terminal 3: RViz
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix bot_urdf)/share/bot_urdf/config/nav.rviz
```

In RViz, use **2D Goal Pose** to send a goal. Fixed frame should be **`map`**.

> Terminals 2 and 3 must source `rtabmap_ws` before `bot_ws` (see [common_errors.md [E5]](common_errors.md)).

Optional launch arguments:

```bash
ros2 launch bot_urdf navigation.launch.py \
  use_sim_time:=true \
  map:=/path/to/kitchen.yaml \
  database_path:=/path/to/rtabmap.db
```

Defaults point to `maps/kitchen.yaml` and `maps/rtabmap.db` in the installed package.

For **real hardware**, set `use_sim_time:=false`, run `realsense.launch.py`, and ensure something publishes `odom` → `base_link` TF (wheel odometry or enable TF on `rgbd_odometry`).

### 5.3 Key topics during navigation

| Topic | Type | Source |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | map_server + rtabmap |
| `/plan` | `nav_msgs/Path` | global planner |
| `/local_plan` | `nav_msgs/Path` | DWB controller |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → Gazebo planar move |
| `/camera/obstacles` | `sensor_msgs/PointCloud2` | obstacles_detection |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz goal tool |

### 5.4 Tuning notes

Nav2's local controller is **DWB** (`dwb_core::DWBLocalPlanner`). It outputs holonomic twists (`vx`, `vy`, `ωz`), which match the simulated planar move plugin.

With a **single forward-facing** D435, side and rear space is often unseen. Params in `nav2_params.yaml` reflect that: conservative speeds, inflated local costmap, and `track_unknown_space: true` on the local costmap so the robot does not drive into unobserved cells. Expect slower motion than a lidar-equipped robot.

## 6. Building the Workspaces

```bash
# Build rtabmap (only needed once)
cd ~/Mechatronics/rtabmap_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build bot_ws (rebuild after any changes to bot_urdf)
cd ~/Mechatronics/bot_ws
source /opt/ros/humble/setup.bash
source ~/Mechatronics/rtabmap_ws/install/setup.bash
colcon build --packages-select bot_urdf

# Source order matters: rtabmap_ws first, then bot_ws
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
```

> If launch fails to find RTAB-Map packages, check [common_errors.md [E5]](common_errors.md).

## Launch Files Reference

| Launch file | Purpose | Key nodes |
|---|---|---|
| `display.launch.py` | Visualize URDF in RViz | robot_state_publisher, joint_state_publisher_gui, rviz2 |
| `gazebo.launch.py` | Simulate robot in Gazebo | gzserver, gzclient, robot_state_publisher, spawn_entity |
| `rviz.launch.py` | RViz with fake odom (early dev only) | robot_state_publisher, cmd_vel_to_odom, rviz2 |
| `realsense.launch.py` | Real D435 camera driver | realsense2_camera_node |
| `mapping.launch.py` | RTAB-Map SLAM | rgbd_odometry, rtabmap |
| `navigation.launch.py` | Nav2 + RTAB-Map localization | map_server, rtabmap, obstacles_detection, Nav2 stack |

## Troubleshooting

See [common_errors.md](common_errors.md) for detailed diagnosis and fixes:

- **[E1]** Gazebo: Meshes not loading / robot invisible after spawn
- **[E2]** Gazebo: Depth camera sensor not publishing topics
- **[E3]** RViz: "Package does not exist" for meshes
- **[E4]** RTAB-Map: "Did not receive data" / no `/map`
- **[E5]** Workspace source order: missing packages or nodes at launch
