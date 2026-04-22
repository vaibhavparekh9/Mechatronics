# Mechatronics

ROS 2 Humble workspace for a custom 4-wheeled mecanum-drive robot equipped with an Intel RealSense D435 depth camera. The stack covers URDF description, Gazebo simulation, RTAB-Map SLAM, and map generation for Nav2.

This can serve as a reference for integrating SLAM and autonomous navigation on any custom robot (not just TurtleBot or Clearpath platforms).

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-desktop`)
- Gazebo Classic (ships with `ros-humble-desktop`)

## Repository Structure

```
Mechatronics/
├── bot_ws/                          # primary ROS 2 workspace
│   └── src/
│       ├── bot_urdf/                # robot description, launches, maps
│       │   ├── urdf/               
│       │   │   └── bot.urdf.xacro   # robot model (xacro)
│       │   ├── meshes/              # STL meshes from SolidWorks
│       │   ├── config/              # joint_names_bot.yaml
│       │   ├── launch/              # all launch files
│       │   ├── worlds/              # Gazebo world files
│       │   └── maps/                # saved maps (.db, .pgm, .yaml)
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
ros2 launch bot_urdf gazebo.launch.py
```

This spawns the robot in an empty Gazebo world with:
- A `libgazebo_ros_planar_move.so` plugin for holonomic drive (mecanum wheels), subscribing to `/cmd_vel` and publishing `/odom`
- A `libgazebo_ros_joint_state_publisher.so` plugin publishing `/joint_states`
- A simulated RealSense D435 depth camera (see section 3)

> If the robot spawns but is invisible (no meshes), check [common_errors.md [E1]](common_errors.md).

**Teleoperation:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The planar move plugin accepts standard `geometry_msgs/Twist` on `/cmd_vel`. You can also add Gazebo models (e.g. the Cafe) to the empty world for a more interesting environment.

**Key topics published:**

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop input |
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

Requires **two terminals** (plus Gazebo already running):

```bash
# Terminal 1: Gazebo
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf gazebo.launch.py

# Terminal 2: RTAB-Map
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
ros2 launch bot_urdf mapping.launch.py use_sim_time:=true
```

`mapping.launch.py` starts two nodes:
- **`rgbd_odometry`**: computes visual odometry from RGB+depth, publishes `/odom`
- **`rtabmap`**: performs SLAM, publishes `/map` (occupancy grid), `/cloud_map` (3D point cloud), and TF

Both subscribe to the D435 topics with `subscribe_depth: True` and `approx_sync: True`. Using `subscribe_rgbd: True` instead would make them expect a single combined `/rgbd_image` topic, which our camera does not publish.

For **real hardware**, change `use_sim_time`:
```bash
ros2 launch bot_urdf mapping.launch.py use_sim_time:=false
```

### 4.3 Visualizing the Map

```bash
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
rviz2 -d ~/Mechatronics/bot_ws/src/bot_urdf/config/mapping.rviz
```

A pre-configured RViz layout is saved at `config/mapping.rviz`. Make sure every terminal has the workspaces sourced, otherwise RViz cannot resolve mesh paths (see [common_errors.md [E3]](common_errors.md)).

Set **Fixed Frame** to `map`, then add:

| Display | Topic | What it shows |
|---|---|---|
| Map | `/map` | 2D occupancy grid |
| PointCloud2 | `/cloud_map` | 3D color point cloud |
| Image | `/camera/camera/image_raw` | Live RGB feed |
| Image | `/camera/camera/depth/image_raw` | Live depth feed |
| TF | (all) | Coordinate frames |
| RobotModel | `/robot_description` | Bot mesh |

Drive the robot around with `teleop_twist_keyboard` to build the map.

**Key RTAB-Map topics:**

| Topic | Type | Content |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 2D grid map (for Nav2) |
| `/odom` | `nav_msgs/Odometry` | Visual odometry |
| `/cloud_map` | `sensor_msgs/PointCloud2` | 3D map |
| `/mapData` | `rtabmap_msgs/MapData` | Full graph data |

### 4.4 Saving the Map

RTAB-Map automatically saves its full database to `~/.ros/rtabmap.db`. This contains all images, depth frames, the pose graph, and loop closures.

**To export a 2D occupancy grid (.pgm + .yaml) for Nav2:**

```bash
rtabmap-databaseViewer /path/to/rtabmap.db
# Then: Edit > Regenerate local grids, then File > Export 2D Grid Map (.pgm)
```

This produces `.pgm` + `.yaml` files that Nav2's `map_server` loads for localization and path planning.

## 5. Building the Workspaces

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

## Launch Files Reference

| Launch file | Purpose | Key nodes |
|---|---|---|
| `display.launch.py` | Visualize URDF in RViz | robot_state_publisher, joint_state_publisher_gui, rviz2 |
| `gazebo.launch.py` | Simulate robot in Gazebo | gzserver, gzclient, robot_state_publisher, spawn_entity |
| `rviz.launch.py` | RViz with wheel odometry | robot_state_publisher, cmd_vel_to_odom, rviz2 |
| `realsense.launch.py` | Real D435 camera driver | realsense2_camera_node |
| `mapping.launch.py` | RTAB-Map SLAM | rgbd_odometry, rtabmap |

## Troubleshooting

See [common_errors.md](common_errors.md) for detailed diagnosis and fixes:

- **[E1]** Gazebo: Meshes not loading / robot invisible after spawn
- **[E2]** Gazebo: Depth camera sensor not publishing topics
- **[E3]** RViz: "Package does not exist" for meshes
