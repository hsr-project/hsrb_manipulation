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
#ifndef HSRB_IK_SOLVER_NODE_SRC_IK_SOLVER_PLUGIN_HPP_
#define HSRB_IK_SOLVER_NODE_SRC_IK_SOLVER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <tmc_ik_solver_node/ik_solver_plugin.hpp>
#include <tmc_robot_kinematics_model/ik_solver.hpp>

namespace hsrb_ik_solver_node {

class IkSolverPluginRobustToBasePositionErrorCommon : public tmc_ik_solver_node::IkSolverPluginRobustToBasePositionError {  // NOLINT
 public:
  explicit IkSolverPluginRobustToBasePositionErrorCommon(
      const std::shared_ptr<tmc_robot_kinematics_model::IKSolver>& ik_solver_impl) : ik_solver_impl_(ik_solver_impl) {}
  virtual ~IkSolverPluginRobustToBasePositionErrorCommon() = default;

 protected:
  // Solve IK based on the target end-effector position/orientation and the cart position
  void SolveIkImpl(const Eigen::Affine3d& origin_to_hand_goal,
                   const tmc_manipulation_types::JointState& initial_joint_state,
                   const Eigen::Affine3d& initial_origin_to_base,
                   double base_pos_x,
                   double base_pos_y,
                   std::vector<tmc_ik_solver_node::IkResult>& results_out) override;

  // Return joint names
  std::vector<std::string> GetJointNames() const override;

 private:
  std::shared_ptr<tmc_robot_kinematics_model::IKSolver> ik_solver_impl_;
};

class HsrbIkSolverPluginRobustToBasePositionError : public IkSolverPluginRobustToBasePositionErrorCommon {
 public:
  HsrbIkSolverPluginRobustToBasePositionError();
  virtual ~HsrbIkSolverPluginRobustToBasePositionError() = default;

 protected:
  // Calculate the range of candidate cart positions based on the target end-effector position/orientation
  bool CalculateBaseCandidateMapSize(const Eigen::Affine3d& origin_to_hand_goal,
                                     std::array<double, 2>& center_out,
                                     std::array<double, 2>& radius_range_out) const override;
};

class HsrcIkSolverPluginRobustToBasePositionError : public IkSolverPluginRobustToBasePositionErrorCommon {
 public:
  HsrcIkSolverPluginRobustToBasePositionError();
  virtual ~HsrcIkSolverPluginRobustToBasePositionError() = default;

 protected:
  // Calculate the range of candidate cart positions based on the target end-effector position/orientation
  bool CalculateBaseCandidateMapSize(const Eigen::Affine3d& origin_to_hand_goal,
                                     std::array<double, 2>& center_out,
                                     std::array<double, 2>& radius_range_out) const override;
};
}  // namespace hsrb_ik_solver_node

#endif  // HSRB_IK_SOLVER_NODE_SRC_IK_SOLVER_PLUGIN_HPP_
