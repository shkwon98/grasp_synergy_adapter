# SPDX-FileCopyrightText: 2026 Sunghyun Kwon
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def _result(node, future, timeout_sec=2.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    assert future.done(), 'ROS request timed out'
    return future.result()


def test_missing_required_parameters_exit_with_error():
    executable = (
        Path(get_package_prefix('grasp_synergy_adapter'))
        / 'lib'
        / 'grasp_synergy_adapter'
        / 'grasp_synergy_adapter'
    )

    result = subprocess.run(
        [str(executable)], capture_output=True, text=True, timeout=5, check=False
    )

    assert result.returncode != 0
    assert 'not initialized' in result.stdout + result.stderr


def test_packaged_profile_exposes_and_remaps_a_piecewise_grasp():
    # ponytail: share one ROS process; split if scenarios need independent execution.
    suffix = str(os.getpid())
    namespace = f'/grasp_adapter_test_{suffix}'
    target = f'/grasp_adapter_target_{suffix}'
    action_name = f'{namespace}/remapped_follow_joint_trajectory'
    executable = (
        Path(get_package_prefix('grasp_synergy_adapter'))
        / 'lib'
        / 'grasp_synergy_adapter'
        / 'grasp_synergy_adapter'
    )
    config = (
        Path(get_package_share_directory('grasp_synergy_adapter'))
        / 'config'
        / 'example.yaml'
    )
    process = subprocess.Popen(
        [
            str(executable),
            '--ros-args',
            '--params-file',
            str(config),
            '-r',
            f'__ns:={namespace}',
            '-r',
            f'target/joint_trajectory:={target}/joint_trajectory',
            '-r',
            f'target/controller_state:={target}/controller_state',
            '-r',
            f'target/follow_joint_trajectory:={target}/follow_joint_trajectory',
            '-r',
            f'pinch_controller/follow_joint_trajectory:={action_name}',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    node = rclpy.create_node(f'adapter_probe_{suffix}')
    target_node = rclpy.create_node(f'target_controller_{suffix}')
    state_publisher = node.create_publisher(
        JointTrajectoryControllerState, f'{target}/controller_state', 1
    )
    command_publisher = node.create_publisher(
        JointTrajectory, f'{namespace}/pinch_controller/joint_trajectory', 1
    )
    outputs = []
    output_subscription = node.create_subscription(
        JointTrajectory,
        f'{target}/joint_trajectory',
        outputs.append,
        1,
    )
    action_client = ActionClient(
        node,
        FollowJointTrajectory,
        action_name,
    )
    target_goals = []
    feedback_messages = []
    target_behavior = 'succeed'
    target_started = threading.Event()
    release_target = threading.Event()

    def execute_target(goal_handle):
        behavior = target_behavior
        target_goals.append(goal_handle.request)
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        if behavior == 'succeed':
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = ['finger_b_joint', 'finger_a_joint']
            feedback.desired.positions = [0.4, 0.8]
            feedback.actual.positions = [0.16, 0.32]
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
        else:
            target_started.set()
            deadline = time.monotonic() + 3.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.01)
            if behavior == 'unconfirmed_cancel':
                release_target.wait(timeout=5.0)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        return result

    target_server = ActionServer(
        target_node,
        FollowJointTrajectory,
        f'{target}/follow_joint_trajectory',
        execute_target,
        goal_callback=lambda _: (
            GoalResponse.REJECT if target_behavior == 'reject' else GoalResponse.ACCEPT
        ),
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup(),
    )
    target_executor = MultiThreadedExecutor(num_threads=2)
    target_executor.add_node(target_node)
    target_spin = threading.Thread(target=target_executor.spin)
    target_spin.start()

    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while (
            command_publisher.get_subscription_count() == 0
            or state_publisher.get_subscription_count() == 0
        ):
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=0.05)

        command = JointTrajectory()
        command.joint_names = ['synergy']
        point = JointTrajectoryPoint()
        point.positions = [1.0]
        point.time_from_start.sec = 10
        command.points = [point]
        command_publisher.publish(command)
        _spin_for(node, 0.15)
        assert not outputs

        state = JointTrajectoryControllerState()
        state.header.stamp = node.get_clock().now().to_msg()
        state.joint_names = ['wrong_joint']
        state.feedback.positions = [0.0]
        state_publisher.publish(state)
        time.sleep(0.05)
        command_publisher.publish(command)
        _spin_for(node, 0.15)
        assert not outputs

        state.joint_names = ['finger_b_joint', 'finger_a_joint']
        state.feedback.positions = [0.16, 0.32]
        state.header.stamp = node.get_clock().now().to_msg()
        state.header.stamp.sec -= 1
        state_publisher.publish(state)
        time.sleep(0.05)
        command_publisher.publish(command)
        _spin_for(node, 0.15)
        assert not outputs

        for _ in range(3):
            state.header.stamp = node.get_clock().now().to_msg()
            state_publisher.publish(state)
            rclpy.spin_once(node, timeout_sec=0.05)

        command_publisher.publish(command)

        deadline = time.monotonic() + 2.0
        while not outputs:
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=0.05)

        action_goal = FollowJointTrajectory.Goal()
        action_goal.trajectory = command

        def send_goal(behavior):
            nonlocal target_behavior
            target_behavior = behavior
            target_started.clear()
            state.header.stamp = node.get_clock().now().to_msg()
            state_publisher.publish(state)
            return _result(node, action_client.send_goal_async(
                action_goal, feedback_callback=feedback_messages.append
            ))

        goal_handle = send_goal('succeed')
        assert goal_handle.accepted
        action_result = _result(node, goal_handle.get_result_async(), timeout_sec=5.0)
        assert action_result.status == GoalStatus.STATUS_SUCCEEDED
        assert action_result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

        goal_handle = send_goal('reject')
        assert goal_handle.accepted
        rejected_result = _result(node, goal_handle.get_result_async(), timeout_sec=5.0)
        assert rejected_result.status == GoalStatus.STATUS_ABORTED
        assert (
            rejected_result.result.error_code
            == FollowJointTrajectory.Result.INVALID_GOAL
        )
        assert 'target controller rejected the goal' in rejected_result.result.error_string

        for behavior in ('cancel', 'unconfirmed_cancel'):
            goal_handle = send_goal(behavior)
            assert goal_handle.accepted
            assert target_started.wait(timeout=2.0)
            if behavior == 'cancel':
                output_count = len(outputs)
                command_publisher.publish(command)
                _spin_for(node, 0.15)
                assert len(outputs) == output_count
            canceled = _result(node, goal_handle.cancel_goal_async())
            assert len(canceled.goals_canceling) == 1
            result = _result(node, goal_handle.get_result_async(), timeout_sec=5.0)
            assert result.status == GoalStatus.STATUS_CANCELED

        assert 'did not confirm a terminal state' in result.result.error_string
        assert not send_goal('succeed').accepted
    finally:
        release_target.set()
        target_executor.shutdown(timeout_sec=5.0)
        target_spin.join(timeout=1.0)
        target_server.destroy()
        target_node.destroy_node()
        action_client.destroy()
        node.destroy_subscription(output_subscription)
        node.destroy_publisher(command_publisher)
        node.destroy_publisher(state_publisher)
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5.0)

    assert process.returncode == 0, output
    trajectory = outputs[-1]
    assert trajectory.joint_names == ['finger_a_joint', 'finger_b_joint']
    assert [list(point.positions) for point in trajectory.points] == [
        [0.8, 0.4],
        [1.0, 1.0],
    ]
    assert target_goals[0].trajectory == trajectory
    assert len(feedback_messages) == 1
    feedback = feedback_messages[0].feedback
    assert feedback.joint_names == ['synergy']
    assert list(feedback.desired.positions) == pytest.approx([0.5])
    assert list(feedback.actual.positions) == pytest.approx([0.2])
    assert list(feedback.error.positions) == pytest.approx([0.3])
