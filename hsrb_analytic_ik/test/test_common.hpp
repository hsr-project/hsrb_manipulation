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
#ifndef HSRB_ANALYTIC_IK_TEST_UTILS_HPP__
#define HSRB_ANALYTIC_IK_TEST_UTILS_HPP__

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Build fails if pinocchio headers are included after boost headers, so include them first.
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>

namespace hsrb_analytic_ik {

// Read as many 1x8 configurations as possible from the file.
void LoadDataFile(const std::string& file_name, std::vector<Eigen::VectorXd>& data_out);
void LoadDataFile(const std::string& file_name, const std::vector<uint32_t>& mask_indices,
                  std::vector<Eigen::VectorXd>& data_out);

class IKTestDriver {
 public:
  using Ptr = std::shared_ptr<IKTestDriver>;

  explicit IKTestDriver(const std::string& robot_description,
                        tmc_manipulation_types::BaseMovementType base_movement_type);
  virtual ~IKTestDriver() = default;

  // Solve FK.
  void SolveFK(const Eigen::VectorXd& config, Eigen::Affine3d& origin_to_end_out);

  // Solve using the provided IKSolver.
  bool SolveIK(const tmc_robot_kinematics_model::IKSolver::Ptr& ik_solver,
               const Eigen::Affine3d& ref_origin_to_end,
               const Eigen::VectorXd& initial_config,
               Eigen::VectorXd& solution_config_out);

  // Solve using the provided IKSolver.
  bool SolveIK(const tmc_robot_kinematics_model::IKSolver::Ptr& ik_solver,
               const Eigen::Affine3d& ref_origin_to_end,
               const Eigen::VectorXd& initial_config,
               std::vector<Eigen::VectorXd>& solution_configs_out);

  // Solve using the internal numerical IKSolver.
  bool SolveNumericIK(const Eigen::Affine3d& ref_origin_to_end,
                      const Eigen::VectorXd& initial_config,
                      Eigen::VectorXd& solution_config_out);

  // Calculate weighted Norm.
  double CalcWeightedNorm(const Eigen::VectorXd& config1,
                          const Eigen::VectorXd& config2) const;

  // Check maximum and minimum values.
  void CheckMinMax(const Eigen::VectorXd& joint) const;

  // Check the movable range of the arm's 5 axes among the 7-axis joint angles.
  // Added reluctantly (During HSR-C testing, random tests caused the 6th axis setting value to fall below the lower limit, so retrieve the lower limit value in such cases).
  bool ArmJointCheckMinMax(const Eigen::VectorXd& joint) const;

 private:
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;
  tmc_robot_kinematics_model::IKSolver::Ptr numeric_solver_;

  tmc_manipulation_types::NameSeq use_name_;
  Eigen::VectorXd weight_vector_;
  Eigen::VectorXd joint_min_;
  Eigen::VectorXd joint_max_;

  tmc_manipulation_types::BaseMovementType base_movement_type_;

  tmc_robot_kinematics_model::IKRequest GenerateRequest(const Eigen::Affine3d& ref_origin_to_end,
                                                        const Eigen::VectorXd& initial_config) const;
};

void RunTest(const std::vector<Eigen::VectorXd>& ref_configs,
             const std::vector<Eigen::VectorXd>& init_configs,
             const IKTestDriver::Ptr& ik_test_driver,
             const tmc_robot_kinematics_model::IKSolver::Ptr& analytic_ik_solver,
             const std::vector<uint32_t>& mask_indices);

void MultiSolutionTest(const Eigen::Affine3d& origin_to_end,
                       uint32_t expected_result_num,
                       const IKTestDriver::Ptr& ik_test_driver,
                       const tmc_robot_kinematics_model::IKSolver::Ptr& analytic_ik_solver);

}  // namespace hsrb_analytic_ik
#endif  // HSRB_ANALYTIC_IK_TEST_UTILS_HPP__
