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

#include "ik_solver_plugin.hpp"

#include <hsrb_analytic_ik/hsrb_ik_solver.hpp>
#include <hsrb_analytic_ik/hsrc_ik_solver.hpp>

namespace {
const char* const kHandName = "hand_palm_link";
const std::vector<std::string> kUseJointNames = {
    "arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
}  // namespace

namespace hsrb_ik_solver_node {

// Solve IK based on the target end-effector position/orientation and the cart position
void IkSolverPluginRobustToBasePositionErrorCommon::SolveIkImpl(
    const Eigen::Affine3d& origin_to_hand_goal,
    __attribute__((unused)) const tmc_manipulation_types::JointState& initial_joint_state,
    __attribute__((unused)) const Eigen::Affine3d& initial_origin_to_base,
    double base_pos_x,
    double base_pos_y,
    std::vector<tmc_ik_solver_node::IkResult>& results_out) {
  tmc_robot_kinematics_model::IKRequest req(tmc_manipulation_types::kRotationZ);
  req.target_frames.resize(1);
  req.target_frames[0].frame_name = kHandName;
  req.target_frames[0].frame_to_end = Eigen::Affine3d::Identity();
  req.target_frames[0].ref_origin_to_end = origin_to_hand_goal;
  req.initial_angle.name = kUseJointNames;
  req.initial_angle.position.resize(kUseJointNames.size());
  req.initial_angle.position.fill(0.0);
  req.origin_to_base = Eigen::Translation3d(base_pos_x, base_pos_y, 0.0) * Eigen::AngleAxisd::Identity();
  // TODO(Takeshita) hsrb_analytic_ikが，サイズ8のweightを求めるという仕様，変なので，どうにかする
  req.weight.resize(8);
  req.weight.fill(1.0);
  req.use_joints = kUseJointNames;

  std::vector<tmc_robot_kinematics_model::IKResponse> responses;
  const auto error_code = ik_solver_impl_->Solve(req, responses);
  if (error_code != tmc_robot_kinematics_model::kSuccess) {
    return;
  }

  tmc_ik_solver_node::IkResult result(base_pos_x, base_pos_y);
  for (const auto& res : responses) {
    result.base_yaw = res.origin_to_base.linear().eulerAngles(0, 1, 2)[2];
    result.joint_positions = res.solution_angle.position;
    results_out.push_back(result);
  }
}

// Return joint names
std::vector<std::string> IkSolverPluginRobustToBasePositionErrorCommon::GetJointNames() const {
  return kUseJointNames;
}


HsrbIkSolverPluginRobustToBasePositionError::HsrbIkSolverPluginRobustToBasePositionError()
    : IkSolverPluginRobustToBasePositionErrorCommon(std::make_shared<hsrb_analytic_ik::HsrbBaseYawIKSolver>()) {}

// Calculate the range of candidate cart positions based on the target end-effector position/orientation
bool HsrbIkSolverPluginRobustToBasePositionError::CalculateBaseCandidateMapSize(
    const Eigen::Affine3d& origin_to_hand_goal,
    std::array<double, 2>& center_out,
    std::array<double, 2>& radius_range_out) const {
  const auto base_position_range = hsrb_analytic_ik::GetHsrbBasePositionRange(origin_to_hand_goal);
  if (base_position_range.radius_min < 0.0) {
    return false;
  }
  center_out = base_position_range.center;
  radius_range_out = {base_position_range.radius_min, base_position_range.radius_max};
  return true;
}

HsrcIkSolverPluginRobustToBasePositionError::HsrcIkSolverPluginRobustToBasePositionError()
    : IkSolverPluginRobustToBasePositionErrorCommon(std::make_shared<hsrb_analytic_ik::HsrcBaseYawIKSolver>()) {}

// Calculate the range of candidate cart positions based on the target end-effector position/orientation
bool HsrcIkSolverPluginRobustToBasePositionError::CalculateBaseCandidateMapSize(
    const Eigen::Affine3d& origin_to_hand_goal,
    std::array<double, 2>& center_out,
    std::array<double, 2>& radius_range_out) const {
  const auto base_position_range = hsrb_analytic_ik::GetHsrcBasePositionRange(origin_to_hand_goal);
  if (base_position_range.radius_min < 0.0) {
    return false;
  }
  center_out = base_position_range.center;
  radius_range_out = {base_position_range.radius_min, base_position_range.radius_max};
  return true;
}

}  // namespace hsrb_ik_solver_node

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(hsrb_ik_solver_node::HsrbIkSolverPluginRobustToBasePositionError,
                       tmc_ik_solver_node::IIkSolverPlugin)
PLUGINLIB_EXPORT_CLASS(hsrb_ik_solver_node::HsrcIkSolverPluginRobustToBasePositionError,
                       tmc_ik_solver_node::IIkSolverPlugin)
