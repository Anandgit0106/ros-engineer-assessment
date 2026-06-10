# nav2_simple_controller

A custom Nav2 controller plugin (Task 2).

This package implements a `nav2_core::Controller` plugin that drives a robot along a planned path using simple proportional heading control, loaded dynamically by the Nav2 Controller Server via `pluginlib`.

---

## Overview

The `SimpleController` plugin:

- Inherits from `nav2_core::Controller`
- Implements all required lifecycle methods: `configure()`, `activate()`, `deactivate()`, `cleanup()`
- Receives a global path via `setPlan()` and computes velocity commands via `computeVelocityCommands()`
- Uses **proportional angular control** — steers toward the nearest waypoint based on heading error
- Uses a **constant linear velocity** for forward motion
- Is discovered at runtime by Nav2 through `pluginlib` and the `plugin.xml` manifest

---

## Control Strategy

```
heading_error  =  atan2(dy, dx) - robot_yaw
angular_vel    =  kp * heading_error
linear_vel     =  constant (0.2 m/s)
```

The robot always steers toward the front waypoint of the global plan with a proportional correction on the yaw error.

---

## Package Structure

```
nav2_simple_controller/
├── include/
│   └── nav2_simple_controller/
│       └── simple_controller.hpp      # Class declaration
├── src/
│   └── simple_controller.cpp          # Plugin implementation
├── plugin.xml                         # pluginlib manifest
├── CMakeLists.txt                     # Build config & plugin registration
└── package.xml                        # ROS2 package metadata
```

---

## Dependencies

| Package | Role |
|---|---|
| `rclcpp` | ROS2 C++ client library |
| `rclcpp_lifecycle` | Lifecycle node support |
| `pluginlib` | Runtime plugin loading |
| `nav2_core` | Base controller interface |
| `nav_msgs` | `nav_msgs::msg::Path` for the global plan |
| `geometry_msgs` | `TwistStamped`, `PoseStamped` |
| `tf2` / `tf2_ros` | Yaw extraction from quaternion |
| `tf2_geometry_msgs` | TF2 geometry conversions |

---

## Build & Install

```bash
# Clone into your ROS2 workspace
cd ~/nav2_ws/src
git clone <this-repo-url>

# Build
cd ~/nav2_ws
colcon build --packages-select nav2_simple_controller

# Source
source install/setup.bash
```

---

## Plugin Registration

The plugin is registered in `plugin.xml`:

```xml
<library path="nav2_simple_controller">
  <class
    type="nav2_simple_controller::SimpleController"
    base_class_type="nav2_core::Controller">
    <description>Simple proportional Nav2 controller</description>
  </class>
</library>
```

And exported in `CMakeLists.txt` via:

```cmake
pluginlib_export_plugin_description_file(nav2_core plugin.xml)
```

This allows the Nav2 Controller Server to discover and load the plugin at runtime without any code changes.

---

## Using the Plugin in Nav2

In your Nav2 `controller_server` parameters YAML:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_simple_controller::SimpleController"
```

---

## Key Implementation Notes

| Method | What it does |
|---|---|
| `configure()` | Sets `kp_ = 1.0`, `linear_velocity_ = 0.2`, stores node handle |
| `activate()` | Logs activation (ready for use) |
| `deactivate()` | Logs deactivation |
| `cleanup()` | Clears the stored global plan |
| `setPlan()` | Stores the incoming global path |
| `computeVelocityCommands()` | Computes `cmd_vel` using heading error proportional control |
| `setSpeedLimit()` | Accepts but ignores speed limit (stub) |
