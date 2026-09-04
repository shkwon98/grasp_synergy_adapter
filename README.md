# grasp_synergy_adapter

`grasp_synergy_adapter` makes a multi-joint robot hand look like a set of
normalized one-DOF grippers. Each configured grasp gets a standard
`JointTrajectoryController`-compatible topic and action endpoint with a
unitless coordinate from `0.0` to `1.0`.

## What it does

- Loads any number of grasp profiles from ROS parameters.
- Maps a normalized coordinate onto a piecewise-linear path through joint space.
- Inserts configured intermediate knots when a command crosses them.
- Projects physical controller feedback back onto the normalized coordinate.
- Relays `FollowJointTrajectory` feedback, cancellation, and results.
- Remaps to any downstream `JointTrajectoryController` namespace.

This package is a regular ROS 2 node. It is not a `ros2_control` controller or
hardware interface.

```text
pinch_controller/joint_trajectory   [synergy: 0.0 ... 1.0]
                         │
                         ▼
              grasp_synergy_adapter
                         │
                         ▼
target/joint_trajectory              [thumb, index, ...]
                         │
                         ▼
          physical JointTrajectoryController
```

Run one adapter node per physical hand or downstream controller.

## Supported platform

ROS 2 Jazzy on Ubuntu 24.04 is the current supported target.

## Versioning and compatibility

This package follows [Semantic Versioning](https://semver.org/). Releases in the
`0.x` series are pre-stable: incompatible changes to the public contract use a
minor version bump and include migration notes, while patch releases preserve
the public contract.

The public contract consists of the executable name, documented ROS interfaces
and QoS, parameters, grasp-profile YAML schema, and documented command behavior.
Headers under `src/` are implementation details and are not installed as a
public C++ API. See
[CHANGELOG.rst](CHANGELOG.rst)
for release changes.

## Installation

### Binary package

Binary packages are not published yet. After the first ROS release, install the
package with:

```bash
sudo apt update
sudo apt install ros-jazzy-grasp-synergy-adapter
```

### From source

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/shkwon98/grasp_synergy_adapter.git \
  ~/ros2_ws/src/grasp_synergy_adapter

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select grasp_synergy_adapter
source install/setup.bash
```

## Quick start

Start the adapter with a grasp profile and remap its three downstream endpoints
to the physical hand controller:

```bash
ros2 run grasp_synergy_adapter grasp_synergy_adapter \
  --ros-args \
  --params-file /absolute/path/to/grasp_profiles.yaml \
  -r target/joint_trajectory:=/hand_controller/joint_trajectory \
  -r target/follow_joint_trajectory:=/hand_controller/follow_joint_trajectory \
  -r target/controller_state:=/hand_controller/controller_state
```

Once the downstream controller publishes fresh state, close the configured
`pinch` grasp:

```bash
ros2 topic pub --once \
  /pinch_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [synergy], points: [{positions: [1.0], time_from_start: {sec: 1}}]}"
```

Command `0.0` to move toward the preparation pose and `1.0` to move toward the
final grasp pose.

## Interfaces

Each entry in `grasp_names` creates two relative client endpoints.

| Endpoint | Type | Direction |
| --- | --- | --- |
| `<grasp>_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Client to adapter |
| `<grasp>_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Client to adapter |

The synergy trajectory must contain exactly one joint named by `synergy_joint`.
Its position is unitless and must stay in `[0, 1]`.

The adapter uses three relative endpoints for the physical controller.

| Endpoint | Type | Direction |
| --- | --- | --- |
| `target/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Adapter to controller |
| `target/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Adapter to controller |
| `target/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | Controller to adapter |

Trajectory and state topics use reliable QoS with depth 1. Remap all three
`target/*` names together.

## Parameters

| Parameter | Type | Unit | Requirement |
| --- | --- | --- | --- |
| `synergy_joint` | string | unitless | Required, nonempty |
| `joints` | string array | joint names | Required, nonempty, unique |
| `state_timeout_sec` | double | seconds | Required, finite, greater than zero |
| `grasp_names` | string array | grasp names | Required, nonempty, unique |
| `knot_names` (per grasp) | string array | knot names | At least two, ordered, unique |
| `coordinate` (per knot) | double | normalized | Finite, strictly increasing from `0.0` to `1.0` |
| `positions` (per knot) | double array | rad or m | One finite value per physical joint |

`synergy_joint` must not duplicate a physical joint. Grasp and knot names must be
valid ROS name tokens.

## Configure a grasp

The
[example profile](config/example.yaml)
documents the complete schema. A minimal three-knot grasp looks like this:

```yaml
/**/grasp_synergy_adapter:
  ros__parameters:
    synergy_joint: synergy
    joints: [finger_a_joint, finger_b_joint]
    state_timeout_sec: 0.5
    grasp_names: [pinch]

    grasps:
      pinch:
        knot_names: [pre_grasp, intermediate, grasp]
        knots:
          pre_grasp:
            coordinate: 0.0
            positions: [0.0, 0.0]
          intermediate:
            coordinate: 0.5
            positions: [0.8, 0.4]
          grasp:
            coordinate: 1.0
            positions: [1.0, 1.0]
```

The `positions` arrays follow `joints` order. Use radians for revolute
joints and metres for prismatic joints. Adjacent knots must describe different
physical poses.

To add another grasp, append its name to `grasp_names` and add a matching nested
block. The node creates the new topic and action endpoints at startup. No code
change is needed.

## Launch integration

The following fragment runs the adapter in `/control/hand` and connects it to
`/hand_controller`:

```python
from launch_ros.actions import Node

adapter = Node(
    package="grasp_synergy_adapter",
    executable="grasp_synergy_adapter",
    name="grasp_synergy_adapter",
    namespace="/control/hand",
    parameters=["/absolute/path/to/grasp_profiles.yaml"],
    remappings=[
        ("target/joint_trajectory", "/hand_controller/joint_trajectory"),
        (
            "target/follow_joint_trajectory",
            "/hand_controller/follow_joint_trajectory",
        ),
        ("target/controller_state", "/hand_controller/controller_state"),
    ],
)
```

With this namespace, the `pinch` topic becomes
`/control/hand/pinch_controller/joint_trajectory`.

## Send commands

Topic command:

```bash
ros2 topic pub --once \
  /control/hand/pinch_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [synergy], points: [{positions: [1.0], time_from_start: {sec: 1}}]}"
```

Action goal:

```bash
ros2 action send_goal \
  /control/hand/pinch_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [synergy], points: [{positions: [1.0], time_from_start: {sec: 1}}]}}" \
  --feedback
```

The endpoint name selects the grasp profile. When changing profiles, first
command `0.0` on the new endpoint to approach its preparation pose.

## Command behavior

The adapter projects the latest physical controller feedback onto the selected
grasp path. It expands each synergy trajectory point into physical joint
positions and inserts every crossed knot with a proportional timestamp. This
works in both opening and closing directions.

Topic commands use last-writer-wins behavior while no action is active. Only one
action may run across all grasp endpoints. While an action is active, the node
rejects other action goals and ignores topic commands.

The downstream JTC must publish complete, current feedback for every joint in
`joints`. The adapter rejects commands until state is available and no
older than `state_timeout_sec`.

## Limitations and safety

- Input trajectories are position-only. Velocity, acceleration, and effort
  fields are rejected.
- Action goals forward `goal_time_tolerance`.
- Path tolerances, goal tolerances, component tolerances, and multi-DOF
  trajectories are not supported.
- The adapter cannot stop another node from commanding the downstream
  controller. Give it exclusive command ownership.
- The downstream controller and hardware limits remain the final safety
  boundary.

Validate every grasp profile without connected hardware before commanding a
robot.

## Test

```bash
colcon test \
  --packages-select grasp_synergy_adapter \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/grasp_synergy_adapter \
  --verbose
```

## Contributing

Bug reports and focused pull requests are welcome. See
[CONTRIBUTING.md](CONTRIBUTING.md)
for the development and review process. Use the
[GitHub issue tracker](https://github.com/shkwon98/grasp_synergy_adapter/issues)
for reproducible problems and proposed changes.

Report suspected security vulnerabilities according to
[SECURITY.md](SECURITY.md),
not through a public issue.

## License

Copyright 2026 Sunghyun Kwon.

Apache License 2.0. See
[LICENSE](LICENSE).
