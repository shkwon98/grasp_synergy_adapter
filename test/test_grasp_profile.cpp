// SPDX-FileCopyrightText: 2026 Sunghyun Kwon
// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "grasp_endpoint.hpp"
#include "grasp_profile.hpp"
#include "trajectory_adapter.hpp"

namespace grasp_synergy_adapter::test
{
namespace
{

const GraspMap kGrasps{
    {"wrap", GraspProfile{{
                 GraspKnot{0.0, {0.0, 0.0}},
                 GraspKnot{0.4, {0.8, 0.4}},
                 GraspKnot{0.7, {0.9, 0.7}},
                 GraspKnot{1.0, {1.0, 1.0}},
             }}},
};

TEST(GraspModelTest, InterpolatesAndProjectsPiecewisePathsForAnyJointCount)
{
    const GraspModel model{kGrasps};

    EXPECT_EQ(model.JointCount(), 2U);
    EXPECT_EQ(model.Interpolate("wrap", 0.2), (JointPositions{0.4, 0.2}));
    EXPECT_EQ(model.Interpolate("wrap", 0.7), (JointPositions{0.9, 0.7}));
    EXPECT_DOUBLE_EQ(*model.Project("wrap", JointPositions{0.9, 0.7}), 0.7);
}

TEST(GraspModelTest, RejectsInvalidProfilesAndEndpointNames)
{
    EXPECT_THROW(static_cast<void>(GraspModel{GraspMap{}}), std::invalid_argument);
    auto invalid = kGrasps;
    invalid.at("wrap").knots[1].positions = invalid.at("wrap").knots[0].positions;
    EXPECT_THROW(static_cast<void>(GraspModel{invalid}), std::invalid_argument);
    EXPECT_EQ(ControllerName("custom_grasp"), "custom_grasp_controller");
    EXPECT_THROW(static_cast<void>(ControllerName("bad/name")), std::invalid_argument);
}

TEST(TrajectoryAdapterTest, InsertsEveryCrossedKnotInBothDirections)
{
    const GraspModel model{kGrasps};
    const std::vector<std::string> joints{"joint_a", "joint_b"};
    trajectory_msgs::msg::JointTrajectory input;
    input.joint_names = {"trigger"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {1.0};
    point.time_from_start.sec = 10;
    input.points.push_back(point);

    const auto forward = ExpandTrajectory(model, "wrap", "trigger", joints, input, 0.2);

    ASSERT_TRUE(forward);
    ASSERT_EQ(forward.trajectory->points.size(), 3U);
    EXPECT_EQ(forward.trajectory->points[0].positions, (JointPositions{0.8, 0.4}));
    EXPECT_EQ(forward.trajectory->points[0].time_from_start.sec, 2);
    EXPECT_EQ(forward.trajectory->points[0].time_from_start.nanosec, 500000000U);
    EXPECT_EQ(forward.trajectory->points[1].positions, (JointPositions{0.9, 0.7}));
    EXPECT_EQ(forward.trajectory->points[1].time_from_start.sec, 6);
    EXPECT_EQ(forward.trajectory->points[1].time_from_start.nanosec, 250000000U);
    EXPECT_EQ(forward.trajectory->points[2].positions, (JointPositions{1.0, 1.0}));
    EXPECT_EQ(forward.trajectory->points[2].time_from_start.sec, 10);

    input.points[0].positions = {0.1};
    input.points[0].time_from_start.sec = 8;
    const auto reverse = ExpandTrajectory(model, "wrap", "trigger", joints, input, 0.9);

    ASSERT_TRUE(reverse);
    ASSERT_EQ(reverse.trajectory->points.size(), 3U);
    EXPECT_EQ(reverse.trajectory->points[0].positions, (JointPositions{0.9, 0.7}));
    EXPECT_EQ(reverse.trajectory->points[0].time_from_start.sec, 2);
    EXPECT_EQ(reverse.trajectory->points[1].positions, (JointPositions{0.8, 0.4}));
    EXPECT_EQ(reverse.trajectory->points[1].time_from_start.sec, 5);
    EXPECT_EQ(reverse.trajectory->points[2].positions, (JointPositions{0.2, 0.1}));
    EXPECT_EQ(reverse.trajectory->points[2].time_from_start.sec, 8);
}

TEST(TrajectoryAdapterTest, RejectsInvalidSynergyTrajectories)
{
    const GraspModel model{kGrasps};
    trajectory_msgs::msg::JointTrajectory input;
    input.joint_names = {"trigger"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.5};
    point.time_from_start.sec = 1;
    input.points.push_back(point);

    EXPECT_FALSE(ExpandTrajectory(model, "wrap", "wrong", {"a", "b"}, input, 0.0));
    input.points[0].velocities = {0.0};
    EXPECT_FALSE(ExpandTrajectory(model, "wrap", "trigger", {"a", "b"}, input, 0.0));
    input.points[0].velocities.clear();
    input.points[0].positions = {1.1};
    EXPECT_FALSE(ExpandTrajectory(model, "wrap", "trigger", {"a", "b"}, input, 0.0));
    input.points[0].positions = {0.6};
    input.points[0].time_from_start.sec = 0;
    input.points[0].time_from_start.nanosec = 1;
    EXPECT_FALSE(ExpandTrajectory(model, "wrap", "trigger", {"a", "b"}, input, 0.0));
}

TEST(TrajectoryAdapterTest, PreservesMultiPointTimingAroundInsertedKnots)
{
    const GraspModel model{kGrasps};
    trajectory_msgs::msg::JointTrajectory input;
    input.joint_names = {"trigger"};
    trajectory_msgs::msg::JointTrajectoryPoint first;
    first.positions = {0.3};
    first.time_from_start.sec = 1;
    trajectory_msgs::msg::JointTrajectoryPoint second;
    second.positions = {0.8};
    second.time_from_start.sec = 6;
    input.points = {first, second};

    const auto output = ExpandTrajectory(model, "wrap", "trigger", {"a", "b"}, input, 0.2);

    ASSERT_TRUE(output);
    ASSERT_EQ(output.trajectory->points.size(), 4U);
    EXPECT_EQ(output.trajectory->points[0].time_from_start.sec, 1);
    EXPECT_EQ(output.trajectory->points[1].time_from_start.sec, 2);
    EXPECT_EQ(output.trajectory->points[2].time_from_start.sec, 5);
    EXPECT_EQ(output.trajectory->points[3].time_from_start.sec, 6);
}

} // namespace
} // namespace grasp_synergy_adapter::test
