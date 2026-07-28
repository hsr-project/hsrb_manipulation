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
/// @brief Plugin to observe the end-effector during trajectory planning for HSR-B

#include "look_hand.hpp"

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <hsr_kinematics/head_kinematics.hpp>

namespace {
// End-effector frame name
const char* const kHandFrameID = "hand_palm_link";
// Camera frame name
const char* const kCameraFrameID = "head_l_stereo_camera_link";
// Head yaw joint name
const char* const kHeadYawJointID = "head_pan_joint";
// Head pitch joint name
const char* const kHeadPitchJointID = "head_tilt_joint";

}  // namespace

namespace hsrb_planner_plugins {

// Function to adjust joints
bool LookHand::Constrain(
    const std::vector<std::string>& use_joints,
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_rplanner::Config& config_in,
    tmc_rplanner::Config& config_out) {
  // Check if the required joint angles are included
  std::vector<std::string>::const_iterator yaw_joint_it(
      std::find(use_joints.begin(), use_joints.end(), kHeadYawJointID));
  if (yaw_joint_it == use_joints.end()) {
    return false;
  }
  std::vector<std::string>::const_iterator pitch_joint_it(
      std::find(use_joints.begin(), use_joints.end(), kHeadPitchJointID));
  if (pitch_joint_it == use_joints.end()) {
    return false;
  }

  // Check if it has sufficient configuration
  if (config_in.size() < use_joints.size()) {
    return false;
  }

  try {
    // Modify the robot model's posture
    tmc_manipulation_types::JointState current_state;
    current_state.name = use_joints;
    current_state.position = config_in.block(0, 0, use_joints.size(), 1);
    robot->SetNamedAngle(current_state);

    // Obtain the target (end-effector) position
    // Here, 'base' means the origin of the space, as per the HsrHeadKinematics specification
    Eigen::Translation3d base_to_hand(
        robot->GetObjectTransform(kHandFrameID).translation());

    // Prepare the class for calculating head posture
    std::vector<std::string> head_joint_names;
    head_joint_names.push_back(kHeadYawJointID);
    head_joint_names.push_back(kHeadPitchJointID);
    hsr_kinematics::HsrHeadKinematics head_kinematics(robot, head_joint_names);

    // Perform calculation
    tmc_manipulation_types::JointState head_state;
    if (head_kinematics.CalculateAngleToGazePoint(base_to_hand, kCameraFrameID, current_state, head_state)) {
      config_out = config_in;
      config_out[std::distance(use_joints.begin(), yaw_joint_it)] =
          head_state.position[0];
      config_out[std::distance(use_joints.begin(), pitch_joint_it)] =
          head_state.position[1];
      return true;
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("hsrb_planner_plugins.look_hand"), "%s", error.what());
  }

  return false;
}

}  // namespace hsrb_planner_plugins
