// SPDX-FileCopyrightText: 2026 Sunghyun Kwon
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "grasp_endpoint.hpp"
#include "grasp_profile.hpp"
#include "trajectory_adapter.hpp"

namespace grasp_synergy_adapter
{

class GraspSynergyAdapterNode final : public rclcpp::Node
{
public:
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    using TrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<TrajectoryAction>;
    using ControllerGoalHandle = rclcpp_action::ClientGoalHandle<TrajectoryAction>;
    using ControllerResult = ControllerGoalHandle::WrappedResult;
    using ControllerState = control_msgs::msg::JointTrajectoryControllerState;

    GraspSynergyAdapterNode()
        : Node("grasp_synergy_adapter"),
          virtual_joint_(RequiredString("virtual_joint")),
          output_joints_(RequiredStringArray("output_joints")),
          state_timeout_sec_(RequiredPositiveDouble("state_timeout_sec")),
          model_(LoadGrasps())
    {
        ValidateJoints();
        if (model_.JointCount() != output_joints_.size())
        {
            throw std::invalid_argument("output_joints size does not match configured knot poses");
        }

        trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "target/joint_trajectory", rclcpp::QoS{1}.reliable());
        controller_client_ =
            rclcpp_action::create_client<TrajectoryAction>(this, "target/follow_joint_trajectory");
        state_subscription_ = create_subscription<ControllerState>(
            "target/controller_state", rclcpp::QoS{1}.reliable(),
            [this](ControllerState::SharedPtr message)
            {
                const std::scoped_lock lock(state_mutex_);
                controller_state_ = std::move(message);
            });

        for (const auto &grasp : model_.GraspNames())
        {
            endpoints_.push_back(CreateEndpoint(grasp));
        }

        RCLCPP_INFO(get_logger(), "Ready with %zu grasp trajectory endpoints on joint '%s'",
                    endpoints_.size(), virtual_joint_.c_str());
    }

private:
    class UncertainTargetState final : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    struct GraspEndpoint
    {
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription;
        rclcpp_action::Server<TrajectoryAction>::SharedPtr action_server;
    };

    std::string RequiredString(const std::string &name)
    {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING);
        const auto value = get_parameter(name).as_string();
        if (value.empty())
        {
            throw std::invalid_argument(name + " must not be empty");
        }
        return value;
    }

    std::vector<std::string> RequiredStringArray(const std::string &name)
    {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
        const auto value = get_parameter(name).as_string_array();
        if (value.empty())
        {
            throw std::invalid_argument(name + " must not be empty");
        }
        return value;
    }

    double RequiredPositiveDouble(const std::string &name)
    {
        declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
        const double value = get_parameter(name).as_double();
        if (!std::isfinite(value) || value <= 0.0)
        {
            throw std::invalid_argument(name + " must be a finite positive value");
        }
        return value;
    }

    GraspModel LoadGrasps()
    {
        const auto names = RequiredStringArray("grasp_names");
        GraspMap grasps;
        for (const auto &name : names)
        {
            static_cast<void>(ControllerName(name));
            const auto knot_names = RequiredStringArray("grasps." + name + ".knot_names");
            std::unordered_set<std::string> unique_knots;
            GraspProfile profile;
            profile.knots.reserve(knot_names.size());
            for (const auto &knot_name : knot_names)
            {
                if (!unique_knots.insert(knot_name).second)
                {
                    throw std::invalid_argument("grasp '" + name + "' has duplicate knot names");
                }
                static_cast<void>(ControllerName(knot_name));
                const auto prefix = "grasps." + name + ".knots." + knot_name;
                declare_parameter(prefix + ".coordinate", rclcpp::ParameterType::PARAMETER_DOUBLE);
                declare_parameter(prefix + ".positions",
                                  rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
                profile.knots.push_back({get_parameter(prefix + ".coordinate").as_double(),
                                         get_parameter(prefix + ".positions").as_double_array()});
            }
            if (!grasps.emplace(name, std::move(profile)).second)
            {
                throw std::invalid_argument("grasp name '" + name + "' is duplicated");
            }
        }
        return GraspModel{std::move(grasps)};
    }

    void ValidateJoints() const
    {
        std::unordered_set<std::string> names;
        names.insert(virtual_joint_);
        for (const auto &joint : output_joints_)
        {
            if (joint.empty() || !names.insert(joint).second)
            {
                throw std::invalid_argument("virtual_joint and output_joints must be unique");
            }
        }
    }

    GraspEndpoint CreateEndpoint(const std::string &grasp)
    {
        const auto controller = ControllerName(grasp);
        GraspEndpoint endpoint;
        endpoint.subscription = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            controller + "/joint_trajectory", rclcpp::QoS{1}.reliable(),
            [this, grasp](trajectory_msgs::msg::JointTrajectory::SharedPtr message)
            { HandleTrajectory(grasp, *message); });
        endpoint.action_server = rclcpp_action::create_server<TrajectoryAction>(
            this, controller + "/follow_joint_trajectory",
            [this, grasp](const rclcpp_action::GoalUUID &uuid,
                          const std::shared_ptr<const TrajectoryAction::Goal> goal)
            { return HandleGoal(grasp, uuid, goal); },
            [this](const std::shared_ptr<TrajectoryGoalHandle> goal_handle)
            { return HandleCancel(goal_handle); },
            [this, grasp](const std::shared_ptr<TrajectoryGoalHandle> goal_handle)
            { HandleAccepted(grasp, goal_handle); });
        return endpoint;
    }

    std::optional<JointPositions> OrderedPositions(const std::vector<std::string> &names,
                                                   const std::vector<double> &positions) const
    {
        if (positions.size() != names.size())
        {
            return std::nullopt;
        }
        JointPositions ordered;
        ordered.reserve(output_joints_.size());
        for (const auto &joint : output_joints_)
        {
            const auto found = std::ranges::find(names, joint);
            if (found == names.end())
            {
                return std::nullopt;
            }
            const double position = positions[static_cast<std::size_t>(found - names.begin())];
            if (!std::isfinite(position))
            {
                return std::nullopt;
            }
            ordered.push_back(position);
        }
        return ordered;
    }

    std::optional<double> CurrentCoordinate(const std::string &grasp) const
    {
        ControllerState::SharedPtr state;
        {
            const std::scoped_lock lock(state_mutex_);
            state = controller_state_;
        }
        if (!state)
        {
            return std::nullopt;
        }
        const double age =
            (get_clock()->now() - rclcpp::Time(state->header.stamp, get_clock()->get_clock_type()))
                .seconds();
        if (age < 0.0 || age > state_timeout_sec_)
        {
            return std::nullopt;
        }
        const auto positions = OrderedPositions(state->joint_names, state->feedback.positions);
        return positions ? model_.Project(grasp, *positions) : std::nullopt;
    }

    TrajectoryExpansion ExpandFromCurrent(
        const std::string &grasp, const trajectory_msgs::msg::JointTrajectory &trajectory) const
    {
        const auto coordinate = CurrentCoordinate(grasp);
        return coordinate ? ExpandTrajectory(model_, grasp, virtual_joint_, output_joints_,
                                             trajectory, *coordinate)
                          : TrajectoryExpansion{std::nullopt,
                                                "target/controller_state is missing or stale"};
    }

    void HandleTrajectory(const std::string &grasp,
                          const trajectory_msgs::msg::JointTrajectory &message)
    {
        if (command_active_.load())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000,
                                 "Ignoring grasp topic while an action is active");
            return;
        }
        const auto trajectory = ExpandFromCurrent(grasp, message);
        if (!trajectory)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000,
                                 "Ignoring invalid '%s' trajectory: %s", grasp.c_str(),
                                 trajectory.error.c_str());
            return;
        }
        if (trajectory_publisher_->get_subscription_count() == 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000,
                                 "target trajectory controller topic is not available");
            return;
        }
        trajectory_publisher_->publish(*trajectory.trajectory);
    }

    rclcpp_action::GoalResponse HandleGoal(const std::string &grasp,
                                           const rclcpp_action::GoalUUID &,
                                           const std::shared_ptr<const TrajectoryAction::Goal> goal)
    {
        const auto trajectory = ExpandFromCurrent(grasp, goal->trajectory);
        if (!goal->multi_dof_trajectory.points.empty() || !goal->path_tolerance.empty() ||
            !goal->component_path_tolerance.empty() || !goal->goal_tolerance.empty() ||
            !goal->component_goal_tolerance.empty() || !trajectory)
        {
            RCLCPP_WARN(get_logger(), "Rejecting invalid '%s' trajectory: %s", grasp.c_str(),
                        trajectory.error.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (command_active_.exchange(true))
        {
            RCLCPP_WARN(get_logger(), "Rejecting '%s' because another action is active",
                        grasp.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_requested_.store(false);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse HandleCancel(
        const std::shared_ptr<TrajectoryGoalHandle> goal_handle)
    {
        const std::scoped_lock lock(terminal_mutex_);
        if (!goal_handle->is_active())
        {
            return rclcpp_action::CancelResponse::REJECT;
        }
        cancel_requested_.store(true);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void HandleAccepted(const std::string &grasp,
                        const std::shared_ptr<TrajectoryGoalHandle> goal_handle)
    {
        worker_ = std::jthread([this, grasp, goal_handle](const std::stop_token &stop_token)
                               { Execute(grasp, goal_handle, stop_token); });
    }

    static TrajectoryAction::Result::SharedPtr ErrorResult(std::string message)
    {
        auto result = std::make_shared<TrajectoryAction::Result>();
        result->error_code = TrajectoryAction::Result::INVALID_GOAL;
        result->error_string = std::move(message);
        return result;
    }

    void Finish(const std::shared_ptr<TrajectoryGoalHandle> &goal_handle,
                rclcpp_action::ResultCode code, TrajectoryAction::Result::SharedPtr result)
    {
        if (!result)
        {
            result = ErrorResult("target controller returned no result");
        }
        using namespace std::chrono_literals;
        while (rclcpp::ok())
        {
            std::unique_lock lock(terminal_mutex_);
            if (cancel_requested_.load() && goal_handle->is_active() &&
                !goal_handle->is_canceling())
            {
                lock.unlock();
                std::this_thread::sleep_for(1ms);
                continue;
            }
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
            }
            else if (goal_handle->is_executing())
            {
                code == rclcpp_action::ResultCode::SUCCEEDED ? goal_handle->succeed(result)
                                                             : goal_handle->abort(result);
            }
            return;
        }
    }

    std::optional<ControllerResult> RunControllerGoal(
        const TrajectoryAction::Goal &goal, const std::stop_token &stop_token,
        const std::function<void(const TrajectoryAction::Feedback &)> &on_feedback)
    {
        using namespace std::chrono_literals;
        if (!controller_client_->wait_for_action_server(2s))
        {
            throw std::runtime_error("target trajectory controller is not available");
        }

        rclcpp_action::Client<TrajectoryAction>::SendGoalOptions options;
        options.feedback_callback =
            [on_feedback](const ControllerGoalHandle::SharedPtr,
                          const std::shared_ptr<const TrajectoryAction::Feedback> feedback)
        { on_feedback(*feedback); };
        auto handle_future = controller_client_->async_send_goal(goal, options);
        while (handle_future.wait_for(20ms) != std::future_status::ready)
        {
            if (!rclcpp::ok() || stop_token.stop_requested())
            {
                return std::nullopt;
            }
        }
        const auto controller_goal_handle = handle_future.get();
        if (!controller_goal_handle)
        {
            throw std::runtime_error("target controller rejected the goal");
        }

        auto result_future = controller_client_->async_get_result(controller_goal_handle);
        const auto started = std::chrono::steady_clock::now();
        double timeout_seconds =
            rclcpp::Duration(goal.trajectory.points.back().time_from_start).seconds() +
            std::max(0.0, rclcpp::Duration(goal.goal_time_tolerance).seconds()) + 5.0;
        if (goal.trajectory.header.stamp.sec != 0 || goal.trajectory.header.stamp.nanosec != 0)
        {
            timeout_seconds += std::max(
                0.0, (rclcpp::Time(goal.trajectory.header.stamp) - get_clock()->now()).seconds());
        }
        const auto timeout = std::chrono::duration<double>(timeout_seconds);
        std::optional<std::chrono::steady_clock::time_point> cancellation_started;
        bool timed_out = false;
        while (result_future.wait_for(20ms) != std::future_status::ready)
        {
            if (!rclcpp::ok())
            {
                return std::nullopt;
            }
            const auto now = std::chrono::steady_clock::now();
            timed_out = timed_out || now - started > timeout;
            if (!cancellation_started &&
                (cancel_requested_.load() || stop_token.stop_requested() || timed_out))
            {
                static_cast<void>(controller_client_->async_cancel_goal(controller_goal_handle));
                cancellation_started = now;
            }
            if (cancellation_started && now - *cancellation_started > 2s)
            {
                throw UncertainTargetState(
                    "target controller did not confirm a terminal state after cancellation");
            }
        }
        auto result = result_future.get();
        if (timed_out)
        {
            throw std::runtime_error("target controller result timed out");
        }
        return result;
    }

    std::optional<double> ProjectFeedback(const std::string &grasp,
                                          const std::vector<std::string> &joint_names,
                                          const std::vector<double> &positions) const
    {
        const auto ordered = OrderedPositions(joint_names, positions);
        return ordered ? model_.Project(grasp, *ordered) : std::nullopt;
    }

    void PublishFeedback(const std::string &grasp,
                         const std::shared_ptr<TrajectoryGoalHandle> &goal_handle,
                         const TrajectoryAction::Feedback &feedback) const
    {
        if (!goal_handle->is_executing())
        {
            return;
        }
        auto output = std::make_shared<TrajectoryAction::Feedback>();
        output->header = feedback.header;
        output->joint_names = {virtual_joint_};
        output->desired.time_from_start = feedback.desired.time_from_start;
        output->actual.time_from_start = feedback.actual.time_from_start;
        output->error.time_from_start = feedback.error.time_from_start;
        const auto desired =
            ProjectFeedback(grasp, feedback.joint_names, feedback.desired.positions);
        const auto actual = ProjectFeedback(grasp, feedback.joint_names, feedback.actual.positions);
        if (desired)
        {
            output->desired.positions = {*desired};
        }
        if (actual)
        {
            output->actual.positions = {*actual};
        }
        if (desired && actual)
        {
            output->error.positions = {*desired - *actual};
        }
        goal_handle->publish_feedback(output);
    }

    void Execute(const std::string &grasp, const std::shared_ptr<TrajectoryGoalHandle> &goal_handle,
                 const std::stop_token &stop_token)
    {
        try
        {
            TrajectoryAction::Goal target_goal = *goal_handle->get_goal();
            const auto trajectory = ExpandFromCurrent(grasp, target_goal.trajectory);
            if (!trajectory)
            {
                throw std::runtime_error(trajectory.error);
            }
            target_goal.trajectory = *trajectory.trajectory;
            const auto result = RunControllerGoal(
                target_goal, stop_token,
                [this, grasp, goal_handle](const TrajectoryAction::Feedback &feedback)
                { PublishFeedback(grasp, goal_handle, feedback); });
            if (result)
            {
                Finish(goal_handle, result->code, result->result);
            }
        }
        catch (const UncertainTargetState &error)
        {
            RCLCPP_ERROR(get_logger(), "%s; command ownership remains latched", error.what());
            Finish(goal_handle, rclcpp_action::ResultCode::ABORTED, ErrorResult(error.what()));
            return;
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(get_logger(), "Grasp trajectory failed: %s", error.what());
            Finish(goal_handle, rclcpp_action::ResultCode::ABORTED, ErrorResult(error.what()));
        }
        command_active_.store(false);
    }

    const std::string virtual_joint_;
    const std::vector<std::string> output_joints_;
    const double state_timeout_sec_;
    const GraspModel model_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    mutable std::mutex state_mutex_;
    ControllerState::SharedPtr controller_state_;
    std::atomic_bool command_active_{false};
    std::atomic_bool cancel_requested_{false};
    std::mutex terminal_mutex_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    rclcpp::Subscription<ControllerState>::SharedPtr state_subscription_;
    rclcpp_action::Client<TrajectoryAction>::SharedPtr controller_client_;
    std::vector<GraspEndpoint> endpoints_;
    std::jthread worker_;
};

} // namespace grasp_synergy_adapter

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<grasp_synergy_adapter::GraspSynergyAdapterNode>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("grasp_synergy_adapter"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
