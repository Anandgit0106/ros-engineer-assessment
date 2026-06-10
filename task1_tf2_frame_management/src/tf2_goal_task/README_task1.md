# tf2_goal_task

A ROS 2 C++ node that manages a three-frame TF tree (`map → odom → base_link`) and dynamically repositions `base_link` in RViz by adjusting the `map → odom` transform whenever a 2D Goal Pose is received.

Built for the ROS Engineer Technical Assessment — Task 1.

---

## What It Does

| State | Behaviour |
|---|---|
| **On startup** | Broadcasts `map → odom → base_link` with both transforms at identity — all frames overlap at the origin in RViz |
| **On `/goal_pose` received** | Updates `map → odom` translation & rotation to match the goal — `base_link` visually jumps to that position |
| **Continuously** | Re-broadcasts both transforms at **10 Hz** so RViz stays in sync |

> The `odom → base_link` transform is **always identity**. All motion is expressed entirely through `map → odom`. This reflects how real robot navigation stacks work — the odometry frame drifts relative to the map, while `base_link` is fixed to `odom`.

---

## TF Tree

```
map
 └── odom          ← adjusted on each /goal_pose
      └── base_link ← always identity (never moves relative to odom)
```

---

## Expected Behaviour in RViz

**Before goal:**
```
map == odom == base_link  (all overlap at origin)
```

**After clicking 2D Goal Pose at (3, 2, yaw=45°):**
```
base_link jumps to position (3.0, 2.0) with 45° rotation
odom shifts with it  (map→odom changed)
base_link rides along since odom→base_link is identity
```

---

## Package Structure

```
tf2_goal_task/
├── src/
│   └── tf_manager_node.cpp    # Node implementation
├── CMakeLists.txt
└── package.xml
```

---

## Dependencies

| Package | Role |
|---|---|
| `rclcpp` | ROS 2 C++ client library |
| `tf2` | Core TF2 types and utilities |
| `tf2_ros` | `TransformBroadcaster` |
| `geometry_msgs` | `PoseStamped`, `TransformStamped` |

---

## Build & Run

```bash
# Clone into your ROS 2 workspace
cd ~/tf2_goal_task_ws/src
git clone <this-repo-url>

# Build
cd ~/tf2_goal_task_ws
colcon build --packages-select tf2_goal_task

# Source
source install/setup.bash

# Run the node
ros2 run tf2_goal_task tf_manager_node
```

---

## Visualise in RViz

1. Launch RViz2:
```bash
rviz2
```

2. In RViz, set **Fixed Frame** to `map`

3. Add a **TF** display — you should see `map`, `odom`, and `base_link` all at the origin

4. Click **2D Goal Pose** in the toolbar and place it anywhere on the grid

5. `base_link` (and `odom`) will instantly jump to that position & orientation

---

## Verify the TF Tree (optional)

```bash
# Print the live TF tree
ros2 run tf2_tools view_frames

# Echo a specific transform
ros2 run tf2_ros tf2_echo map base_link
```

---

## Key Implementation Details

| Detail | How it's handled |
|---|---|
| **Parent → child direction** | `map → odom → base_link` set via `header.frame_id` / `child_frame_id` |
| **Quaternion orientation** | Full quaternion from RViz `PoseStamped` used directly — no yaw-only shortcut |
| **10 Hz broadcasting** | `create_wall_timer(100ms, ...)` drives `broadcastTransforms()` |
| **Goal callback** | Updates `map_to_odom_` translation + rotation; `odom_to_base_` stays identity |
| **Timestamps** | Both transforms stamped with `get_clock()->now()` on every broadcast cycle |
