# Robot Perception Manager — Home Assignment 2 (ROS 2 Jazzy)

A ROS 2 perception-management system with custom messages, services, and actions.  
The project publishes simulated object detections, supports changing the confidence
threshold while detection is running, broadcasts the camera TF, and includes a
properly preemptable action server.

## Architecture

```text
                                      /set_confidence
                               ┌─────────────────────────┐
                               │                         ▼
Action client ── /start_detection ──> perception_manager
                                      │       │
                                      │       ├── publishes /detections
                                      │       │      (Detection.msg at ~10 Hz)
                                      │       │
                                      │       ├── sends action feedback
                                      │       │      current_fps
                                      │       │      detections_so_far
                                      │       │
                                      │       └── returns action result
                                      │              total_detections
                                      │              success
                                      │
                                      └── confidence_threshold
                                             loaded from YAML and
                                             updateable mid-action

camera_tf_broadcaster
        │
        └── /tf_static
              base_link ──> camera_optical_frame
```

## Main Features

- Publishes simulated detections on `/detections` at approximately 10 Hz.
- Uses the action goal's `target_class` as the detection class.
- Generates random confidence values between the current threshold and `1.0`.
- Generates valid randomized bounding boxes.
- Publishes action feedback approximately every `0.2 s`.
- Supports normal action cancellation.
- Updates `confidence_threshold` through `/set_confidence` while an action is running.
- Broadcasts the static transform `base_link -> camera_optical_frame`.
- Loads node parameters from a YAML file.
- Launches both nodes from one Python launch file.
- Supports proper action preemption:
  - a new goal cancels the active goal;
  - the old client receives a `CANCELED` result;
  - the new goal starts immediately;
  - the node does not restart.

## Package Layout

```text
ros2-robot-perception-manager/
├── src/
│   ├── robot_perception_interfaces/
│   │   ├── action/
│   │   │   └── StartDetection.action
│   │   ├── msg/
│   │   │   └── Detection.msg
│   │   ├── srv/
│   │   │   └── SetConfidenceThreshold.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── robot_perception_manager/
│       ├── config/
│       │   └── perception_params.yaml
│       ├── launch/
│       │   └── perception_system.launch.py
│       ├── src/
│       │   ├── perception_manager.cpp
│       │   └── camera_tf_broadcaster.cpp
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md
```

## Custom Interfaces

### `Detection.msg`

```text
string class_name
float32 confidence
float32 x_min
float32 y_min
float32 x_max
float32 y_max
builtin_interfaces/Time stamp
```

### `SetConfidenceThreshold.srv`

```text
float32 threshold
---
bool success
string message
```

### `StartDetection.action`

```text
string target_class
---
int32 total_detections
bool success
---
float32 current_fps
int32 detections_so_far
```

## Build

From the repository root:

```bash
cd ~/ros2-robot-perception-manager

colcon build --symlink-install --packages-select \
robot_perception_interfaces \
robot_perception_manager

source install/setup.bash
```

Every new terminal must source the workspace:

```bash
source ~/ros2-robot-perception-manager/install/setup.bash
```

## Run

Launch both nodes and load the YAML parameters:

```bash
ros2 launch robot_perception_manager perception_system.launch.py
```

The launch output should confirm that these nodes started:

```text
/perception_manager
/camera_tf_broadcaster
```

## Verification

Use separate terminals while the launch file remains running.

### 1. Check the available nodes and interfaces

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

Expected important names:

```text
/perception_manager
/camera_tf_broadcaster
/detections
/set_confidence
/start_detection
```

### 2. Send an action goal and watch feedback

```bash
ros2 action send_goal /start_detection \
robot_perception_interfaces/action/StartDetection \
"{target_class: 'person'}" \
--feedback
```

Expected feedback:

```text
current_fps: 10.0
detections_so_far: ...
```

### 3. Observe the published detections

```bash
ros2 topic echo /detections
```

The messages should use the requested target class and contain confidence values,
bounding-box coordinates, and timestamps.

To check the publishing rate:

```bash
ros2 topic hz /detections
```

Expected rate:

```text
approximately 10 Hz
```

### 4. Change the confidence threshold during an active goal

While the action is still running:

```bash
ros2 service call /set_confidence \
robot_perception_interfaces/srv/SetConfidenceThreshold \
"{threshold: 0.85}"
```

Expected response:

```text
success: true
message: Confidence threshold updated successfully
```

Confirm the parameter value:

```bash
ros2 param get /perception_manager confidence_threshold
```

New detections should have confidence values between `0.85` and `1.0`.

### 5. Cancel an action

While the action command is running, press:

```text
Ctrl+C
```

The action should finish with:

```text
Goal finished with status: CANCELED
```

### 6. Verify the TF tree

```bash
ros2 run tf2_tools view_frames
```

Open the generated PDF:

```bash
xdg-open "$(ls -t frames*.pdf | head -n 1)"
```

The TF tree should contain:

```text
base_link ──> camera_optical_frame
```

The transform can also be inspected directly:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

## Bonus — Verify Action Preemption

Start a first goal:

```bash
ros2 action send_goal /start_detection \
robot_perception_interfaces/action/StartDetection \
"{target_class: 'person'}" \
--feedback
```

While it is still running, send a second goal from another terminal:

```bash
ros2 action send_goal /start_detection \
robot_perception_interfaces/action/StartDetection \
"{target_class: 'car'}" \
--feedback
```

Expected behavior:

1. The first goal finishes with status `CANCELED`.
2. Its result contains the number of detections produced before cancellation.
3. The second goal starts immediately.
4. `/detections` changes from class `person` to class `car`.
5. The launch process and both nodes remain running.

## Parameters

The parameters are stored in:

```text
robot_perception_manager/config/perception_params.yaml
```

They include:

- `confidence_threshold` for the perception manager;
- `tx`, `ty`, `tz` for camera translation;
- `roll`, `pitch`, `yaw` for camera orientation.

The values are loaded by `perception_system.launch.py`, avoiding hardcoded runtime
configuration.

## Stop the System

Return to the launch terminal and press:

```text
Ctrl+C
```
