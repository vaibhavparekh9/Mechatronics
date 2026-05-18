# Common Errors

## [E1] Gazebo: Meshes not loading / Robot invisible after spawn

### Symptom

- `spawn_entity.py` reports "Successfully spawned entity" but the robot is invisible in Gazebo
- Gazebo logs (in `~/.gazebo/server-*/default.log`) show errors like:

```
[Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://bot_urdf/meshes/base_link.STL]
[Wrn] [SystemPaths.cc:459] File or path does not exist [""] [model://bot_urdf/meshes/base_link.STL]
[Err] [MeshShape.cc:64] Failed to find mesh file [model://bot_urdf/meshes/base_link.STL]
```

### Root Cause

When `spawn_entity.py` sends a URDF to Gazebo, Gazebo's internal URDF-to-SDF converter rewrites all `package://` mesh URIs to `model://` URIs. For example:

```
package://bot_urdf/meshes/base_link.STL  →  model://bot_urdf/meshes/base_link.STL
```

Gazebo then tries to find the mesh by searching every directory listed in the `GAZEBO_MODEL_PATH` environment variable for `bot_urdf/meshes/base_link.STL`. If the package's install share directory isn't registered there, Gazebo can't resolve the path and silently fails to render the mesh. The physics entity still exists (plugins load, topics publish), but visually the robot is completely invisible.

### Solution

Register the parent of the package share directory in `GAZEBO_MODEL_PATH` so that `model://bot_urdf/meshes/...` resolves to `<share>/bot_urdf/meshes/...`.

**In the launch file** (immediate effect):

```python
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('bot_urdf')
gazebo_models_path = os.path.join(pkg_share, os.pardir)
existing = os.environ.get('GAZEBO_MODEL_PATH', '')
model_path = gazebo_models_path + (':' + existing if existing else '')

set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)
# Add set_gazebo_model_path as the FIRST action in LaunchDescription
```

**In `package.xml`** (takes effect after sourcing the workspace):

```xml
<export>
  <build_type>ament_cmake</build_type>
  <gazebo_ros gazebo_model_path="${prefix}/.."/>
</export>
```

Both approaches ensure Gazebo can resolve `model://bot_urdf/meshes/*` to the actual installed mesh files.

## [E2] Gazebo: Depth camera sensor not publishing topics

### Symptom

- The camera mesh is visible on the robot in Gazebo, but `ros2 topic list | grep camera` shows nothing.
- No errors in the terminal -- Gazebo silently drops the sensor.

### Root Cause (1): Wrong plugin library name

In ROS 2 Humble's `gazebo_ros_pkgs`, the separate `libgazebo_ros_depth_camera.so` does not exist. All camera types (RGB, depth, multi) are handled by the unified `libgazebo_ros_camera.so`. If the URDF/xacro references the non-existent library, Gazebo silently fails to load it.

**Fix:** Use `libgazebo_ros_camera.so` with sensor type `depth`:

```xml
<sensor name="my_depth_cam" type="depth">
  ...
  <plugin name="depth_cam_plugin" filename="libgazebo_ros_camera.so">
    ...
  </plugin>
</sensor>
```

### Root Cause (2): Sensor on a link that gets merged away

Gazebo Classic converts URDF to SDF at spawn time. During this conversion, links connected by fixed joints are merged into the parent link. If a `<gazebo reference="some_link">` targets a link that has no visual/collision/inertia (e.g. `camera_depth_frame` from the `realsense2_description` D435 macro), that link gets absorbed into its parent and the `<gazebo reference>` can't find it. The sensor definition is silently dropped.

**Fix:** Attach the sensor to the link that has geometry (e.g. `camera_link` instead of `camera_depth_frame`). The `<frame_name>` inside the plugin still correctly stamps published data with the optical frame:

```xml
<gazebo reference="camera_link">
  <sensor name="realsense_d435" type="depth">
    ...
    <plugin name="depth_cam_plugin" filename="libgazebo_ros_camera.so">
      <frame_name>camera_depth_optical_frame</frame_name>
      ...
    </plugin>
  </sensor>
</gazebo>
```

**How to verify:** Convert the expanded URDF to SDF and check that the sensor survived:

```bash
xacro my_robot.urdf.xacro > /tmp/expanded.urdf
gz sdf -p /tmp/expanded.urdf | grep '<sensor name'
```

If the sensor is missing from the SDF output, the `<gazebo reference>` target link was merged away.

## [E3] RViz: "Package does not exist" for meshes

### Symptom

RViz shows errors like:

```
[ERROR] [rviz2]: Error retrieving file [package://bot_urdf/meshes/base_link.STL]: Package [bot_urdf] does not exist
[ERROR] [rviz2]: Could not load resource [package://realsense2_description/meshes/d435.dae]
```

### Root Cause

The terminal running `rviz2` was not sourced with the workspace overlay. RViz resolves `package://` URIs through the ROS `ament_index`, which only knows about packages from sourced workspaces. Unlike Gazebo (which uses `GAZEBO_MODEL_PATH`), RViz has no fallback path mechanism.

### Solution

Source the workspace before launching RViz:

```bash
source ~/Mechatronics/bot_ws/install/setup.bash
rviz2
```

## [E4] RTAB-Map: "Did not receive data" / no `/map`

### Symptom

- `rgbd_odometry` and/or `rtabmap` log warnings like:

```
rgbd_odometry: Did not receive data since 5 seconds!
rgbd_odometry subscribed to:
  /rgbd_image
```

- RViz **Map** display shows "No map received"
- Depth image in RViz may appear black

### Root Cause

RTAB-Map nodes default to **`subscribe_rgbd: True`**, which expects a single combined **`/rgbd_image`** topic. The RealSense D435 (sim or hardware) publishes **separate** RGB and depth topics (`/camera/camera/image_raw`, `/camera/camera/depth/image_raw`, etc.), not a fused `/rgbd_image`.

If launch files or manual `ros2 run` commands omit the correct mode, nodes subscribe to the wrong topic and never receive data.

### Solution

Use **`subscribe_depth: True`** (not `subscribe_rgbd: True`) and remap the individual camera topics. This is already set in `mapping.launch.py` and `navigation.launch.py`:

```python
parameters=[{
    'subscribe_depth': True,
    'approx_sync': True,
}],
remappings=[
    ('rgb/image', '/camera/camera/image_raw'),
    ('rgb/camera_info', '/camera/camera/camera_info'),
    ('depth/image', '/camera/camera/depth/image_raw'),
],
```

Verify camera topics are publishing before starting RTAB-Map:

```bash
ros2 topic hz /camera/camera/image_raw
ros2 topic hz /camera/camera/depth/image_raw
```

## [E5] Workspace source order: missing packages or nodes at launch

### Symptom

- `ros2 launch bot_urdf mapping.launch.py` or `navigation.launch.py` fails with **package not found** (e.g. `rtabmap_odom`, `rtabmap_slam`, `rtabmap_util`)
- `ros2 pkg prefix bot_urdf` works in one terminal but RTAB-Map executables are not found
- Nodes start but behave as if RTAB-Map was never built

### Root Cause

`bot_ws` depends on packages built in **`rtabmap_ws`** as an underlay. Sourcing only `bot_ws` (or sourcing in the wrong order) leaves RTAB-Map packages off the `AMENT_PREFIX_PATH`.

### Solution

Always source in this order in **every terminal** that runs mapping or navigation:

```bash
source /opt/ros/humble/setup.bash
source ~/Mechatronics/rtabmap_ws/install/setup.bash
source ~/Mechatronics/bot_ws/install/setup.bash
```

Gazebo-only workflows can use `bot_ws` alone, but any terminal running RTAB-Map or Nav2 with RTAB-Map localization needs both overlays.
