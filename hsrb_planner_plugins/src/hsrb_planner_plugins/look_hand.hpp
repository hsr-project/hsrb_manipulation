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
#ifndef HSRB_PLANNER_PLUGINS_LOOK_HAND_HPP_
#define HSRB_PLANNER_PLUGINS_LOOK_HAND_HPP_

#include <string>
#include <vector>

#include <tmc_robot_planner/configuration_constraint.hpp>

namespace hsrb_planner_plugins {

/// Plugin to observe the end-effector during trajectory planning for HSR-B
class LookHand : public tmc_robot_planner::IConfigurationConstraint {
 public:
  /// Constructor
  LookHand() {}
  /// Destructor
  ~LookHand() {}

  /// Function to modify joints
  virtual bool Constrain(
      const std::vector<std::string>& use_joints,
      const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
      const tmc_rplanner::Config& config_in,
      tmc_rplanner::Config& config_out);

 private:
  // Copy prohibited
  LookHand(const LookHand& rhs);
  LookHand& operator=(const LookHand& rhs);
};

}  // namespace hsrb_planner_plugins
#endif  // HSRB_PLANNER_PLUGINS_LOOK_HAND_HPP_
