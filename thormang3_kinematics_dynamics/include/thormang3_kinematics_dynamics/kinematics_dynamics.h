/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * kinematcis_dynamics.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_
#define THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <vector>
#include <eigen3/Eigen/Eigen>

#include "kinematics_dynamics_define.h"
#include "link_data.h"

namespace thormang3
{

enum TreeSelect {
  Manipulation,
  Walking,
  WholeBody
};

class KinematicsDynamics
{

public:
  KinematicsDynamics();
  ~KinematicsDynamics();
  KinematicsDynamics(TreeSelect tree);

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMassCenter(int joint_id);
  Eigen::MatrixXd calcCenterOfMass(Eigen::MatrixXd mc);

  //add by matsushima
  void calcTotalMass(void);
  void calcMassCenter(void);
  void calcCenterOfMass(void);
  void calcInertiaC(void);
  void calcInertiaC(int id);
  Eigen::MatrixXd calcCenterOfMass(int id);
  Eigen::Matrix3d cross(Eigen::Vector3d x);
  Eigen::MatrixXd calcInertiaB(int id);
  /* Eigen::MatrixXd calcInertiaB(int id,Eigen::Vector3d com); */
  /* Eigen::MatrixXd calcInertiaC(void); */
  /* Eigen::MatrixXd calcInertiaC(int id); */
  void calcForwardKinematics4rmc(int joint_id);
  Eigen::MatrixXd pInv(Eigen::MatrixXd A);
  void rmcInit(void);
  void rmcCalcParam(void);
  bool calcRMC(Eigen::MatrixXd S,
               Eigen::VectorXd P_ref,
               Eigen::VectorXd L_ref,
               std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref);

  void calcJointsCenterOfMass(int joint_id);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation , int max_iter, double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err);

  // with weight
  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err, Eigen::MatrixXd weight);

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);

  LinkData *thormang3_link_data_ [ ALL_JOINT_ID + 1 ];

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;

  double m_tilde;//全質量 matsushima
  std::vector<int> MH_id_from = {1,2,15,16};
  std::vector<int> MH_id_to = {35,34,45,46};
  std::vector<int> MH_id = {35,34,45,46};
  Eigen::MatrixXd MH_free;
  std::vector<int> idx_;
  Eigen::VectorXi idx;
  Eigen::Vector3d c_tilde_j = Eigen::VectorXd::Zero(3);
  double m_tilde_j = 0.0;
  Eigen::Matrix3d I_tilde_j = Eigen::MatrixXd::Zero(3,3);
  Eigen::Vector3d r_j;//
  Eigen::Vector3d m_j;
  Eigen::Vector3d h_j;  
  Eigen::Vector3d c_tilde;//全重心位置
  Eigen::VectorXd th; //現在角度
  Eigen::Vector3d p_b; //現在のベースリンクの位置
  Eigen::Vector3d r_B_c;//ボディから全重心への位置ベクトル
  
};

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_ */
