// SPDX-FileCopyrightText: 2026 Sunghyun Kwon
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace grasp_synergy_adapter
{

using JointPositions = std::vector<double>;

struct GraspKnot
{
    double coordinate;
    JointPositions positions;
};

struct GraspProfile
{
    std::vector<GraspKnot> knots;
};

using GraspMap = std::unordered_map<std::string, GraspProfile>;

class GraspModel final
{
public:
    explicit GraspModel(GraspMap grasps)
        : grasps_(std::move(grasps)),
          joint_count_(Validate(grasps_))
    {
    }

    [[nodiscard]] std::size_t JointCount() const noexcept
    {
        return joint_count_;
    }

    [[nodiscard]] auto GraspNames() const noexcept
    {
        return std::views::keys(grasps_);
    }

    [[nodiscard]] const GraspProfile *Find(const std::string &name) const
    {
        const auto grasp = grasps_.find(name);
        return grasp == grasps_.end() ? nullptr : &grasp->second;
    }

    [[nodiscard]] std::optional<JointPositions> Interpolate(const std::string &name,
                                                            double coordinate) const
    {
        const auto *profile = Find(name);
        if (profile == nullptr || !std::isfinite(coordinate) || coordinate < 0.0 ||
            coordinate > 1.0)
        {
            return std::nullopt;
        }

        const auto upper =
            std::ranges::upper_bound(profile->knots, coordinate, {}, &GraspKnot::coordinate);
        if (upper == profile->knots.begin())
        {
            return upper->positions;
        }
        if (upper == profile->knots.end())
        {
            return profile->knots.back().positions;
        }

        const auto &lower = *std::prev(upper);
        const double alpha =
            (coordinate - lower.coordinate) / (upper->coordinate - lower.coordinate);
        JointPositions positions(joint_count_);
        std::ranges::transform(lower.positions, upper->positions, positions.begin(),
                               [alpha](double from, double to)
                               { return std::lerp(from, to, alpha); });
        return positions;
    }

    [[nodiscard]] std::optional<double> Project(const std::string &name,
                                                const JointPositions &positions) const
    {
        const auto *profile = Find(name);
        if (profile == nullptr || positions.size() != joint_count_ ||
            !std::ranges::all_of(positions, [](double value) { return std::isfinite(value); }))
        {
            return std::nullopt;
        }

        double best_coordinate = 0.0;
        double best_distance = std::numeric_limits<double>::infinity();
        for (std::size_t segment = 1; segment < profile->knots.size(); ++segment)
        {
            const auto &from = profile->knots[segment - 1];
            const auto &to = profile->knots[segment];
            double numerator = 0.0;
            double denominator = 0.0;
            for (std::size_t joint = 0; joint < joint_count_; ++joint)
            {
                const double delta = to.positions[joint] - from.positions[joint];
                numerator += (positions[joint] - from.positions[joint]) * delta;
                denominator += delta * delta;
            }
            const double alpha = std::clamp(numerator / denominator, 0.0, 1.0);
            double distance = 0.0;
            for (std::size_t joint = 0; joint < joint_count_; ++joint)
            {
                const double projected =
                    std::lerp(from.positions[joint], to.positions[joint], alpha);
                const double error = positions[joint] - projected;
                distance += error * error;
            }
            const double coordinate = std::lerp(from.coordinate, to.coordinate, alpha);
            if (distance < best_distance ||
                (distance == best_distance && coordinate < best_coordinate))
            {
                best_distance = distance;
                best_coordinate = coordinate;
            }
        }
        return best_coordinate;
    }

private:
    static std::size_t Validate(const GraspMap &grasps)
    {
        if (grasps.empty())
        {
            throw std::invalid_argument("at least one grasp profile is required");
        }

        std::size_t joint_count = 0;
        for (const auto &[name, profile] : grasps)
        {
            if (profile.knots.size() < 2)
            {
                throw std::invalid_argument("grasp '" + name + "' requires at least two knots");
            }
            if (profile.knots.front().coordinate != 0.0 || profile.knots.back().coordinate != 1.0)
            {
                throw std::invalid_argument("grasp '" + name + "' must start at 0 and end at 1");
            }
            if (joint_count == 0)
            {
                joint_count = profile.knots.front().positions.size();
            }
            if (joint_count == 0)
            {
                throw std::invalid_argument("grasp profiles require at least one output joint");
            }

            for (std::size_t index = 0; index < profile.knots.size(); ++index)
            {
                const auto &knot = profile.knots[index];
                if (!std::isfinite(knot.coordinate) || knot.positions.size() != joint_count ||
                    !std::ranges::all_of(knot.positions,
                                         [](double value) { return std::isfinite(value); }))
                {
                    throw std::invalid_argument("grasp '" + name + "' has an invalid knot");
                }
                if (index == 0)
                {
                    continue;
                }
                const auto &previous = profile.knots[index - 1];
                if (knot.coordinate <= previous.coordinate || knot.positions == previous.positions)
                {
                    throw std::invalid_argument("grasp '" + name +
                                                "' knots must progress in coordinate and pose");
                }
            }
        }
        return joint_count;
    }

    GraspMap grasps_;
    std::size_t joint_count_;
};

} // namespace grasp_synergy_adapter
