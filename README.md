# Grasp synergy adapter

[![Jazzy CI](https://github.com/shkwon98/grasp_synergy_adapter/actions/workflows/ci.yaml/badge.svg)](https://github.com/shkwon98/grasp_synergy_adapter/actions/workflows/ci.yaml)

`grasp_synergy_adapter` maps a normalized one-joint grasp command to a configured
multi-joint hand trajectory. Each grasp profile appears as a standard
`JointTrajectoryController`-compatible topic and action interface.

The package is a regular ROS 2 node, not a `ros2_control` controller or hardware
interface. It forwards expanded commands to one physical
`JointTrajectoryController` (JTC).

## Interface

For every name in `grasp_names`, the node creates these relative endpoints:

```text
<grasp>_controller/joint_trajectory
<grasp>_controller/follow_joint_trajectory
```

They use the standard ROS 2 types:

- `trajectory_msgs/msg/JointTrajectory`
- `control_msgs/action/FollowJointTrajectory`

The virtual trajectory must contain only `virtual_joint`. Its position is a
unitless grasp coordinate in `[0, 1]`, where `0` and `1` are the first and last
configured knots. Intermediate values follow the piecewise-linear path through
the configured knots.

The node connects to one downstream JTC through three relative names:

```text
target/joint_trajectory
target/follow_joint_trajectory
target/controller_state
```

Remap all three names to the physical controller. Run one adapter node per
physical hand or downstream controller.

## Installation

ROS 2 Jazzy on Ubuntu 24.04 is the current supported platform.

Once the package is available from the ROS package repository, install it with:

```bash
sudo apt update
sudo apt install ros-jazzy-grasp-synergy-adapter
```

Until then, or to use the latest source, clone it into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/shkwon98/grasp_synergy_adapter.git ~/ros2_ws/src/grasp_synergy_adapter
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select grasp_synergy_adapter
source install/setup.bash
```

## Configure grasp profiles

Create a parameter file using the nested structure below. The
[example profile](config/example.yaml) contains the same schema with comments.

```yaml
/**/grasp_synergy_adapter:
  ros__parameters:
    virtual_joint: trigger
    output_joints: [finger_a_joint, finger_b_joint]
    state_timeout_sec: 0.5
    grasp_names: [pinch]

    grasps:
      pinch:
        knot_names: [open, half, closed]
        knots:
          open:
            coordinate: 0.0
            positions: [0.0, 0.0]
          half:
            coordinate: 0.5
            positions: [0.8, 0.4]
          closed:
            coordinate: 1.0
            positions: [1.0, 1.0]
```

Configuration rules:

- `virtual_joint`, `output_joints`, `state_timeout_sec`, and `grasp_names` are
  required.
- Joint names and grasp names must be nonempty and unique.
- Every grasp needs at least two knots.
- Knot coordinates must be finite, strictly increasing, start at `0.0`, and end
  at `1.0`.
- Each `positions` array must be finite and match `output_joints` in order and
  length. Revolute positions use radians and prismatic positions use metres.
- Adjacent knots must have different physical positions.

## Connect a physical controller

This launch fragment runs the adapter in `/control/hand` and connects it to a
controller at `/hand_controller`:

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

The downstream JTC must publish complete, current feedback for every joint in
`output_joints`. Commands are rejected until this state is available and no
older than `state_timeout_sec`.

## Send commands

With the configuration and namespace above, send a topic command with:

```bash
ros2 topic pub --once \
  /control/hand/pinch_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [trigger], points: [{positions: [1.0], time_from_start: {sec: 1}}]}"
```

Send the equivalent action goal with:

```bash
ros2 action send_goal \
  /control/hand/pinch_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [trigger], points: [{positions: [1.0], time_from_start: {sec: 1}}]}}" \
  --feedback
```

The endpoint selects the grasp profile for each command. When changing profiles,
command `0.0` on the new endpoint to reach its preparation pose before closing
toward `1.0`.

## Mapping and command behavior

The adapter projects the latest physical controller feedback onto the selected
grasp path. It then expands each virtual point into physical joint positions.
Any configured knots crossed on the way are inserted into the outgoing
trajectory with proportional timestamps, in either direction.

Topic commands use last-writer-wins behavior while no action is active. Only one
action may be active across all grasp endpoints. During an action, other action
goals are rejected and topic commands are ignored. Give the adapter exclusive
command ownership of the downstream JTC; it cannot prevent another node from
publishing directly to that controller.

Input trajectories are position-only. Velocity, acceleration, and effort fields
are rejected. Action goals forward `goal_time_tolerance`, but path tolerances,
goal tolerances, component tolerances, and multi-DOF trajectory fields are not
supported. Action feedback is projected back to the normalized virtual joint,
and downstream results and cancellation are relayed to the client.

Use the downstream controller and hardware limits as the final safety boundary.
Validate every profile without connected hardware before commanding a robot.

## Test

```bash
colcon test --packages-select grasp_synergy_adapter --event-handlers console_direct+
colcon test-result --test-result-base build/grasp_synergy_adapter --verbose
```

## License

This package is licensed under the Apache License 2.0. See [LICENSE](LICENSE).
