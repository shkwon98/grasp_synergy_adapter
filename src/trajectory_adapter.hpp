// SPDX-FileCopyrightText: 2026 Sunghyun Kwon
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "grasp_profile.hpp"

namespace grasp_synergy_adapter
{

struct TrajectoryExpansion
{
    std::optional<trajectory_msgs::msg::JointTrajectory> trajectory;
    std::string error;

    explicit operator bool() const noexcept
    {
        return trajectory.has_value();
    }
};

inline std::int64_t DurationNanoseconds(const builtin_interfaces::msg::Duration &duration)
{
    return static_cast<std::int64_t>(duration.sec) * 1'000'000'000LL + duration.nanosec;
}

inline builtin_interfaces::msg::Duration DurationFromNanoseconds(std::int64_t nanoseconds)
{
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
    duration.nanosec = static_cast<std::uint32_t>(nanoseconds % 1'000'000'000LL);
    return duration;
}

inline TrajectoryExpansion ExpandTrajectory(const GraspModel &model, const std::string &grasp,
                                            std::string_view synergy_joint,
                                            const std::vector<std::string> &joints,
                                            const trajectory_msgs::msg::JointTrajectory &input,
                                            double start_coordinate)
{
    const auto *profile = model.Find(grasp);
    if (profile == nullptr)
    {
        return {std::nullopt, "unknown grasp"};
    }
    if (joints.size() != model.JointCount())
    {
        return {std::nullopt, "joints size does not match the grasp profile"};
    }
    if (!std::isfinite(start_coordinate) || start_coordinate < 0.0 || start_coordinate > 1.0)
    {
        return {std::nullopt, "current grasp coordinate is invalid"};
    }
    if (input.joint_names.size() != 1 || input.joint_names.front() != synergy_joint ||
        input.points.empty())
    {
        return {std::nullopt, "expected one non-empty synergy-joint trajectory"};
    }

    trajectory_msgs::msg::JointTrajectory output;
    output.header = input.header;
    output.joint_names = joints;
    double previous_coordinate = start_coordinate;
    std::int64_t previous_time = 0;
    for (const auto &input_point : input.points)
    {
        const std::int64_t target_time = DurationNanoseconds(input_point.time_from_start);
        if (input_point.positions.size() != 1 || !input_point.velocities.empty() ||
            !input_point.accelerations.empty() || !input_point.effort.empty() ||
            !std::isfinite(input_point.positions.front()) || input_point.positions.front() < 0.0 ||
            input_point.positions.front() > 1.0 || target_time <= previous_time)
        {
            return {std::nullopt,
                    "points must be position-only, in [0, 1], with increasing positive time"};
        }

        const double target_coordinate = input_point.positions.front();
        const bool forward = target_coordinate > previous_coordinate;
        const auto append_knot = [&](const GraspKnot &knot)
        {
            const bool crossed =
                forward
                    ? knot.coordinate > previous_coordinate && knot.coordinate < target_coordinate
                    : knot.coordinate < previous_coordinate && knot.coordinate > target_coordinate;
            if (!crossed)
            {
                return true;
            }
            const double fraction =
                (knot.coordinate - previous_coordinate) / (target_coordinate - previous_coordinate);
            const auto knot_time =
                previous_time +
                static_cast<std::int64_t>(std::llround(fraction * (target_time - previous_time)));
            if ((!output.points.empty() &&
                 knot_time <= DurationNanoseconds(output.points.back().time_from_start)) ||
                knot_time <= previous_time || knot_time >= target_time)
            {
                return false;
            }
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = knot.positions;
            point.time_from_start = DurationFromNanoseconds(knot_time);
            output.points.push_back(std::move(point));
            return true;
        };
        const bool knots_valid =
            forward ? std::ranges::all_of(profile->knots, append_knot)
                    : std::ranges::all_of(profile->knots | std::views::reverse, append_knot);
        if (!knots_valid)
        {
            return {std::nullopt, "trajectory timing is too short for crossed knots"};
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = *model.Interpolate(grasp, target_coordinate);
        point.time_from_start = input_point.time_from_start;
        output.points.push_back(std::move(point));
        previous_coordinate = target_coordinate;
        previous_time = target_time;
    }
    return {std::move(output), {}};
}

} // namespace grasp_synergy_adapter
