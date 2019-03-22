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
 * ThorManipulation.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: Zerom
 */

#ifndef THORMANG3_WHEEL_MODULE_WHEEL_MODULE_H_
#define THORMANG3_WHEEL_MODULE_WHEEL_MODULE_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

#include "wheel_module_state.h"

namespace thormang3
{

class WheelJointData
{
public:
  double position_;
  double velocity_;
  double effort_;

  int p_gain_;
  int i_gain_;
  int d_gain_;
};

class WheelJointState
{
public:
  WheelJointData curr_joint_state_[MAX_JOINT_ID + 1];
  WheelJointData goal_joint_state_[MAX_JOINT_ID + 1];
  WheelJointData fake_joint_state_[MAX_JOINT_ID + 1];
};

class WheelModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<WheelModule>
{
public:
  WheelModule();
  virtual ~WheelModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  /* ROS Topic Callback Functions */
  void wheelPoseMsgCallback(const std_msgs::String::ConstPtr& msg);

  /* ROS Calculation Functions */
  void wheelPoseTrajGenerateProc();

  void poseGenerateProc(Eigen::MatrixXd joint_angle_pose);
  void poseGenerateProc(std::map<std::string, double>& joint_angle_pose);

  /* Parameter */
  WheelModuleState  *wheel_module_state_;
  WheelJointState   *joint_state_;

private:
  void queueThread();

  void setCtrlModule(std::string module);
  void parseWheelPoseData(const std::string &path);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(const std::string done_msg);

  int           control_cycle_msec_;

  boost::thread queue_thread_;
  boost::thread tra_gene_tread_;

  // ros::Publisher      send_tra_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::Publisher movement_done_pub_;

  std::map<std::string, int> joint_name_to_id_;

  bool has_goal_joints_;
  bool wheel_pose_only_;
};

}

#endif /* THORMANG3_WHEEL_MODULE_WHEEL_MODULE_H_ */
