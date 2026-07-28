/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
/// @brief Test of the plugin that observes the end-effector during trajectory planning for HSR-B

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_planner/configuration_constraint.hpp>

#include "../src/hsrb_planner_plugins/look_hand.hpp"

namespace hsrb_planner_plugins {

// Load test
TEST(LookHandTest, LoadPlugin) {
  pluginlib::ClassLoader<tmc_robot_planner::IConfigurationConstraint>
      plugin_loader("tmc_robot_planner", "tmc_robot_planner::IConfigurationConstraint");
  ASSERT_NO_THROW(plugin_loader.createSharedInstance("hsrb_planner_plugins/LookHand"));
}

// Normal case test
TEST(LookHandTest, NormalConstrain) {
  // Preparation
  std::string robot_description = tmc_manipulation_tests::hsrb::GetUrdf();
  ASSERT_NE(robot_description, "");

  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr model;
  ASSERT_NO_THROW(model.reset(
      new tmc_robot_kinematics_model::PinocchioWrapper(robot_description)));

  tmc_robot_planner::IConfigurationConstraint::Ptr constrain;
  ASSERT_NO_THROW(constrain.reset(new LookHand));

  std::vector<std::string> use_joints;
  use_joints.push_back("arm_lift_joint");
  use_joints.push_back("arm_flex_joint");
  use_joints.push_back("arm_roll_joint");
  use_joints.push_back("wrist_flex_joint");
  use_joints.push_back("wrist_roll_joint");
  use_joints.push_back("head_pan_joint");
  use_joints.push_back("head_tilt_joint");

  // The size of config may be larger than the size of use_joints
  tmc_rplanner::Config config;
  config.resize(8);
  config[0] = 0.0;
  config[1] = -1.57;
  config[2] = 0.0;
  config[3] = 0.0;
  config[4] = 0.0;
  config[5] = 0.0;
  config[6] = 0.0;

  // Execution
  tmc_rplanner::Config result;
  EXPECT_TRUE(constrain->Constrain(use_joints, model, config, result));

  // If the arm is extended straight horizontally, it should cover this range
  EXPECT_LT(0.0, result[5]);
  EXPECT_GT(0.3, result[5]);
  EXPECT_LT(-0.9, result[6]);
  EXPECT_GT(-0.5, result[6]);
}

// Abnormal case test
TEST(LookHandTest, AbnormalConstrain) {
  // Preparation
  std::string robot_description = tmc_manipulation_tests::hsrb::GetUrdf();
  ASSERT_NE(robot_description, "");

  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr model;
  ASSERT_NO_THROW(model.reset(
      new tmc_robot_kinematics_model::PinocchioWrapper(robot_description)));

  tmc_robot_planner::IConfigurationConstraint::Ptr constrain;
  ASSERT_NO_THROW(constrain.reset(new LookHand));

  std::vector<std::string> use_joints;
  use_joints.push_back("arm_lift_joint");
  use_joints.push_back("arm_flex_joint");
  use_joints.push_back("arm_roll_joint");
  use_joints.push_back("wrist_flex_joint");
  use_joints.push_back("wrist_roll_joint");
  use_joints.push_back("head_pan_joint");
  use_joints.push_back("head_tilt_joint");

  tmc_rplanner::Config config;
  config.resize(7);
  config[0] = 0.0;
  config[1] = -1.57;
  config[2] = 0.0;
  config[3] = 0.0;
  config[4] = 0.0;
  config[5] = 0.0;
  config[6] = 0.0;

  // head_tilt_joint is missing
  std::vector<std::string> invalid_joints(use_joints);
  invalid_joints.pop_back();
  tmc_rplanner::Config result;
  EXPECT_FALSE(constrain->Constrain(invalid_joints, model, config, result));

  // head_pan_joint is missing
  invalid_joints.pop_back();
  invalid_joints.push_back("head_tilt_joint");
  EXPECT_FALSE(constrain->Constrain(invalid_joints, model, config, result));
}

}  // namespace hsrb_planner_plugins

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
