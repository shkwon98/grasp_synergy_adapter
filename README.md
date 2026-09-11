# grasp_synergy_adapter

A ROS 2 adapter for controlling multi-finger hands through predefined grasp
synergies. Each grasp exposes a `JointTrajectoryController`-compatible topic
and action interface with a single normalized coordinate:

- `0.0`: preparation posture
- `1.0`: final grasp posture

## What it does

Grasp profiles define piecewise-linear paths through joint space. The adapter
converts normalized commands into joint trajectories, preserves intermediate
knots during opening and closing, and projects measured joint feedback onto
the selected grasp path.

```text
Grasp command: profile + normalized coordinate
                      │
                      ▼
           grasp_synergy_adapter
                      │
                      ▼
      ros2_control JointTrajectoryController
                      │
                      ▼
                 Robot hand
```

The adapter runs as a regular ROS 2 node, not a `ros2_control` controller or
hardware interface. Run one instance per physical hand or downstream controller.

## Installation

Supported platform: **ROS 2 Jazzy on Ubuntu 24.04**.

CI also targets Humble (Ubuntu 22.04), Kilted (Ubuntu 24.04), and Lyrical
(Ubuntu 26.04), with Rolling on Ubuntu 26.04 for forward compatibility.
Each distribution is checked on amd64 and arm64 using the same `main` branch.
Local amd64 builds, unit and integration tests, and lint checks pass on all
five distributions. GitHub CI confirmation, including arm64, is pending.
CI coverage does not imply that binary packages have been released.

Binary packages are not yet published. Build from source:

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

For another distribution, replace `jazzy` in the setup command with its name
and use the corresponding Ubuntu version listed above.

## Quick start

### 1. Define a grasp profile

Save the following as `grasp_profiles.yaml`, replacing the joint names and
positions with values appropriate for your hand:

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

Each `positions` array follows the order of `joints`. Use radians for revolute
joints and metres for prismatic joints. Adjacent knots must describe different
physical poses.

To configure additional grasps, add their names to `grasp_names` and provide
matching blocks under `grasps`. Each profile gets its own topic and action
endpoints at startup.

See [config/example.yaml](config/example.yaml) for the annotated schema.

### 2. Start the adapter

With your hand's `JointTrajectoryController` running, start the adapter and
remap all three downstream endpoints. This example uses `/hand_controller`:

```bash
ros2 run grasp_synergy_adapter grasp_synergy_adapter \
  --ros-args \
  --params-file /absolute/path/to/grasp_profiles.yaml \
  -r target/joint_trajectory:=/hand_controller/joint_trajectory \
  -r target/follow_joint_trajectory:=/hand_controller/follow_joint_trajectory \
  -r target/controller_state:=/hand_controller/controller_state
```

The controller must publish fresh state for every configured joint before
the adapter accepts commands.

### 3. Send a grasp command

Move to the final posture of the `pinch` profile over one second:

```bash
ros2 topic pub --once \
  /pinch_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [synergy], points: [{positions: [1.0], time_from_start: {sec: 1}}]}"
```

Command `0.0` to move toward the preparation posture, or an intermediate value
to move partway along the grasp path.

For action feedback, cancellation, and results, use `FollowJointTrajectory`:

```bash
ros2 action send_goal \
  /pinch_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [synergy], points: [{positions: [1.0], time_from_start: {sec: 1}}]}}" \
  --feedback
```

The endpoint selects the grasp profile. When switching profiles, first command
`0.0` on the new endpoint to approach its preparation posture.

## Interfaces

Each entry in `grasp_names` creates two relative client endpoints:

| Endpoint | Type | Direction |
| --- | --- | --- |
| `<grasp>_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Client → adapter |
| `<grasp>_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Client → adapter |

Input trajectories must contain exactly one joint, named by `synergy_joint`,
with a unitless position in `[0, 1]`.

The adapter connects to the downstream controller through three relative
endpoints:

| Endpoint | Type | Direction |
| --- | --- | --- |
| `target/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Adapter → controller |
| `target/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Adapter → controller |
| `target/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | Controller → adapter |

Trajectory and state topics use reliable QoS with depth 1. Remap all three
`target/*` endpoints together.

## Parameters

| Parameter | Type | Requirement |
| --- | --- | --- |
| `synergy_joint` | string | Required, nonempty; must not duplicate a physical joint name |
| `joints` | string array | Required, nonempty, unique physical joint names |
| `state_timeout_sec` | double | Required, finite, greater than zero; seconds |
| `grasp_names` | string array | Required, nonempty, unique grasp names |
| `knot_names` (per grasp) | string array | At least two, ordered, unique knot names |
| `coordinate` (per knot) | double | Finite, strictly increasing from `0.0` to `1.0` |
| `positions` (per knot) | double array | One finite value per physical joint; radians or metres |

Grasp and knot names must be valid ROS name tokens.

## Command behavior

The adapter projects the latest measured joint positions onto the selected
grasp path. It expands each input trajectory point into physical joint
positions and inserts every crossed knot with a proportional timestamp,
in both opening and closing directions.

Topic commands use last-writer-wins behavior while no action is active.
Only one action may run across all grasp endpoints. During an active action,
the adapter rejects other action goals and ignores topic commands.

Commands are rejected until downstream feedback is available for every
configured joint and is no older than `state_timeout_sec`.

## Launch integration

<details>
<summary>Example: run under /control/hand and connect to /hand_controller</summary>

Add this node to your launch description:

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

Client endpoints inherit the node namespace. In this example, the `pinch`
topic is `/control/hand/pinch_controller/joint_trajectory` and its action is
`/control/hand/pinch_controller/follow_joint_trajectory`.

</details>

## Limitations and safety

- Input trajectories are position-only. Velocity, acceleration, and effort
  fields are rejected.
- Action goals forward `goal_time_tolerance`. Path tolerances, goal tolerances,
  component tolerances, and multi-DOF trajectories are not supported.
- The adapter cannot prevent other nodes from commanding the downstream
  controller. Give it exclusive command ownership.
- The downstream controller and hardware limits remain the final safety
  boundary.

Validate every grasp profile without connected hardware before commanding
a robot.

## Compatibility

This package follows [Semantic Versioning](https://semver.org/).
During the pre-stable `0.x` series, incompatible changes use a minor version
bump and include migration notes. Patch releases preserve the public contract.

The public contract covers the executable name, documented ROS interfaces
and QoS, parameters, grasp-profile YAML schema, and command behavior.
Headers under `src/` are internal and are not installed as a public C++ API.

See [CHANGELOG.rst](CHANGELOG.rst) for release changes.

## Contributing

Bug reports and focused pull requests are welcome.
See [CONTRIBUTING.md](CONTRIBUTING.md) for build, test, and contribution
instructions, or open an [issue](https://github.com/shkwon98/grasp_synergy_adapter/issues)
for reproducible problems and proposed changes.

Report security vulnerabilities through the process in
[SECURITY.md](SECURITY.md), rather than a public issue.

## Citation

If you use this package in your research, please cite:

```bibtex
@software{kwon2026grasp_synergy_adapter,
  author = {Kwon, Sunghyun},
  title  = {{Grasp Synergy Adapter}: A {ROS 2} Interface for Synergy-Based Hand Control},
  url    = {https://github.com/shkwon98/grasp_synergy_adapter},
  year   = {2026}
}
```

## License

Copyright 2026 Sunghyun Kwon.
Licensed under the [Apache License 2.0](LICENSE).
