/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_manipulation_tests/configs.hpp>

#include "../src/ik_solver_plugin.hpp"

namespace {
constexpr double kEpsilon = 2.0e-2;
const std::vector<std::string> kUseJointNames = {
    "arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
}  // namespace

namespace hsrb_ik_solver_node {

TEST(IkSolverPluginLoadingTest, LoadPlugin) {
  pluginlib::ClassLoader<tmc_ik_solver_node::IIkSolverPlugin> loader(
      "tmc_ik_solver_node", "tmc_ik_solver_node::IIkSolverPlugin");
  loader.createSharedInstance("hsrb_ik_solver_node::HsrbIkSolverPluginRobustToBasePositionError");
  loader.createSharedInstance("hsrb_ik_solver_node::HsrcIkSolverPluginRobustToBasePositionError");
}

template<typename SolverType>
class IkSolverPluginTest : public ::testing::Test {
 protected:
  void SetUp() override;

  tmc_robot_collision_detector::RobotCollisionDetector::Ptr collision_detector_;
};

typedef ::testing::Types<HsrbIkSolverPluginRobustToBasePositionError,
                         HsrcIkSolverPluginRobustToBasePositionError> SolverTypes;
TYPED_TEST_CASE(IkSolverPluginTest, SolverTypes);

template<typename SolverType>
void IkSolverPluginTest<SolverType>::SetUp() {
  const auto robot_description = tmc_manipulation_tests::hsrb::GetUrdf();
  const auto robot_collision_pair = tmc_manipulation_tests::hsrb::GetCollisionConfig();

  // Since we are using the HSR-B model, it does not strictly match with HSR-C.
  // However, the error can be contained by setting kEpsilon to 2.0e-2, so we won't push too hard.
  collision_detector_ = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
      robot_description, robot_collision_pair, "ODE");
}

TYPED_TEST(IkSolverPluginTest, SolveIk) {
  auto node = rclcpp::Node::make_shared("test");
  node->declare_parameter("map_convolution_type", "tmc_ik_solver_node::ManhattanDistanceMapConvolution");

  const auto solver = std::make_shared<TypeParam>();
  EXPECT_TRUE(solver->Init(node));

  // If we only hit the outer interface, we can only check whether the solution looks plausible.
  // It doesn't make much sense as a test, but we'll run the software anyway.
  std::vector<tmc_manipulation_msgs::msg::IkResult> results;
  solver->SolveIk(Eigen::Translation3d(0.0, 0.0, 1.0) * Eigen::Quaterniond(0.707, 0.0, 0.707, 0.0),
                  tmc_manipulation_types::OccupancyGrid(),
                  this->collision_detector_,
                  tmc_manipulation_types::JointState(),
                  Eigen::Affine3d::Identity(),
                  results);
  EXPECT_FALSE(results.empty());

  for (const auto& result : results) {
    Eigen::Affine3d origin_to_base;
    tf2::fromMsg(result.origin_to_base, origin_to_base);
    this->collision_detector_->SetRobotTransform(origin_to_base);

    tmc_manipulation_types::JointState joint_state;
    joint_state.name = result.joint_names;
    joint_state.position = Eigen::Map<const Eigen::VectorXd>(&result.joint_positions[0],
                                                             result.joint_positions.size());
    this->collision_detector_->SetRobotNamedAngle(joint_state);

    const auto origin_to_hand_goal = this->collision_detector_->GetObjectTransform("hand_palm_link");
    EXPECT_NEAR(origin_to_hand_goal.translation().x(), 0.0, kEpsilon);
    EXPECT_NEAR(origin_to_hand_goal.translation().y(), 0.0, kEpsilon);
    EXPECT_NEAR(origin_to_hand_goal.translation().z(), 1.0, kEpsilon);
    EXPECT_NEAR(Eigen::Quaterniond(origin_to_hand_goal.linear()).w(), 0.71, kEpsilon);
    EXPECT_NEAR(Eigen::Quaterniond(origin_to_hand_goal.linear()).x(), 0.0, kEpsilon);
    EXPECT_NEAR(Eigen::Quaterniond(origin_to_hand_goal.linear()).y(), 0.71, kEpsilon);
    EXPECT_NEAR(Eigen::Quaterniond(origin_to_hand_goal.linear()).z(), 0.0, kEpsilon);
  }
}

}  // namespace hsrb_ik_solver_node

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
