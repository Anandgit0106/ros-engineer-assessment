# agv_charging — ROS 2 Package

**Task 3: Action Server vs Service Client — Timeout Behaviour**

Demonstrates the difference between a ROS 2 Service and a ROS 2 Action for a long-running warehouse AGV task: navigating to a charging station (30 seconds of simulated travel).

---

## Package Structure

```
agv_charging/
├── CMakeLists.txt
├── package.xml
├── README.md
├── srv/
│   └── GoToCharger.srv        # Custom service interface (Version A)
├── action/
│   └── GoToCharger.action     # Custom action interface (Version B)
└── src/
    ├── service_server.cpp     # Version A — Service server (sleeps 30s)
    ├── service_client.cpp     # Version A — Service client (5s timeout)
    ├── action_server.cpp      # Version B — Action server (feedback every 1s)
    └── action_client.cpp      # Version B — Action client (async, no timeout)
```

---

## Interface Definitions

### `srv/GoToCharger.srv`
```
bool go
---
bool success
float32 travel_time
string message
```

### `action/GoToCharger.action`
```
bool go
---
bool success
float32 travel_time
string message
---
float32 distance_remaining
```

---

## Build Instructions

```bash
cd ~/charging_task_ws
colcon build --packages-select agv_charging
source install/setup.bash
```

---

## Running Version A — Service (Timeout Demo)

Open two terminals:

**Terminal 1 — Start the service server:**
```bash
ros2 run agv_charging service_server
```

**Terminal 2 — Run the service client:**
```bash
ros2 run agv_charging service_client
```

### Expected Output (Terminal 2)
```
[client] Requesting charging station trip via SERVICE...
[client] Waiting for response (5s timeout)...
[client] ✘ TIMED OUT — service did not respond within 5 seconds!
```

**What happens:** The service server sleeps for 30 seconds before replying. The client's `spin_until_future_complete()` blocks and expires after 5 seconds — the goal is abandoned with no result and no feedback.

---

## Running Version B — Action (No Timeout)

Open two terminals:

**Terminal 1 — Start the action server:**
```bash
ros2 run agv_charging action_server
```

**Terminal 2 — Run the action client:**
```bash
ros2 run agv_charging action_client
```

### Expected Output (Terminal 2)
```
[client] Goal accepted — receiving feedback...
[feedback] Distance remaining: 29.0 m
[feedback] Distance remaining: 28.0 m
[feedback] Distance remaining: 27.0 m
... (continues every second for 30 seconds) ...
[feedback] Distance remaining: 1.0 m
[feedback] Distance remaining: 0.0 m
[client] ✔ Result: Arrived at charging station! Travel time: 30.0s
```

**What happens:** The client sends the goal asynchronously and registers callbacks. It stays connected for the full 30 seconds, receiving distance feedback every second, and finally receives the result — no timeout.

---

## Key Concept: Service vs Action

| Feature | Service (Version A) | Action (Version B) |
|---|---|---|
| **Blocking?** | Yes — client blocks until response or timeout | No — fully asynchronous callbacks |
| **Timeout risk?** | Yes — hard timeout, goal abandoned | No — client stays connected indefinitely |
| **Feedback during execution?** | ❌ Not supported | ✅ Published every second |
| **Cancellation?** | ❌ Cannot cancel mid-execution | ✅ Can cancel at any time |
| **Best used for** | Fast queries (< 1s) | Long-running tasks (navigation, manipulation) |

---

## When to Choose Actions Over Services

Use an **Action** when the task:
- Takes more than ~1 second to complete
- Needs to report **intermediate progress** (e.g. distance remaining, % complete)
- Must be **cancellable** mid-execution (e.g. obstacle detected, new priority)
- Runs asynchronously while the robot does other work in parallel

Use a **Service** when the task:
- Is a quick, synchronous query (e.g. get battery level, set a parameter)
- Always returns in well under a second
- Does not need feedback or cancellation

**Real robotics examples for Actions:** `NavigateToPose`, move arm to waypoint, execute pick-and-place, dock to charging station (this package).

**Real robotics examples for Services:** Get map, trigger an E-stop, query sensor status, save a waypoint.

---

## Dependencies

- `rclcpp`
- `rclcpp_action`
- `rosidl_default_generators` / `rosidl_default_runtime`
