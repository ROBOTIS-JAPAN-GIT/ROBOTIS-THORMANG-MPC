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
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: Zerom
 */

#include <stdio.h>
#include "thormang3_wheel_module/wheel_module.h"

#define EXT_PORT_DATA_1 "external_port_data_1"
#define EXT_PORT_DATA_2 "external_port_data_2"
#define EXT_PORT_DATA_3 "external_port_data_3"
#define EXT_PORT_DATA_4 "external_port_data_4"

using namespace thormang3;

WheelModule::WheelModule()
  : control_cycle_msec_(0),
    has_goal_joints_(false),
    wheel_pose_only_(false)
{
  enable_       = false;
  module_name_  = "wheel_module";
  control_mode_ = robotis_framework::PositionControl;

  /* arm */
  result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();

  /* gripper */
  result_["r_arm_grip"]   = new robotis_framework::DynamixelState();
  result_["l_arm_grip"]   = new robotis_framework::DynamixelState();

  /* body */
  result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* leg */
  result_["r_leg_hip_y"]  = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"]  = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"]  = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p"]   = new robotis_framework::DynamixelState();
  result_["r_leg_an_p"]   = new robotis_framework::DynamixelState();
  result_["r_leg_an_r"]   = new robotis_framework::DynamixelState();

  result_["l_leg_hip_y"]  = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"]  = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"]  = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p"]   = new robotis_framework::DynamixelState();
  result_["l_leg_an_p"]   = new robotis_framework::DynamixelState();
  result_["l_leg_an_r"]   = new robotis_framework::DynamixelState();

  /* head */
  result_["head_y"]       = new robotis_framework::DynamixelState();
  result_["head_p"]       = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"]  = 1;
  joint_name_to_id_["l_arm_sh_p1"]  = 2;
  joint_name_to_id_["r_arm_sh_r"]   = 3;
  joint_name_to_id_["l_arm_sh_r"]   = 4;
  joint_name_to_id_["r_arm_sh_p2"]  = 5;
  joint_name_to_id_["l_arm_sh_p2"]  = 6;
  joint_name_to_id_["r_arm_el_y"]   = 7;
  joint_name_to_id_["l_arm_el_y"]   = 8;
  joint_name_to_id_["r_arm_wr_r"]   = 9;
  joint_name_to_id_["l_arm_wr_r"]   = 10;
  joint_name_to_id_["r_arm_wr_y"]   = 11;
  joint_name_to_id_["l_arm_wr_y"]   = 12;
  joint_name_to_id_["r_arm_wr_p"]   = 13;
  joint_name_to_id_["l_arm_wr_p"]   = 14;

  /* leg */
  joint_name_to_id_["r_leg_hip_y"]  = 15;
  joint_name_to_id_["l_leg_hip_y"]  = 16;
  joint_name_to_id_["r_leg_hip_r"]  = 17;
  joint_name_to_id_["l_leg_hip_r"]  = 18;
  joint_name_to_id_["r_leg_hip_p"]  = 19;
  joint_name_to_id_["l_leg_hip_p"]  = 20;
  joint_name_to_id_["r_leg_kn_p"]   = 21;
  joint_name_to_id_["l_leg_kn_p"]   = 22;
  joint_name_to_id_["r_leg_an_p"]   = 23;
  joint_name_to_id_["l_leg_an_p"]   = 24;
  joint_name_to_id_["r_leg_an_r"]   = 25;
  joint_name_to_id_["l_leg_an_r"]   = 26;

  /* body */
  joint_name_to_id_["torso_y"]      = 27;

  /* head */
  joint_name_to_id_["head_y"]       = 28;
  joint_name_to_id_["head_p"]       = 29;

  /* gripper */
  joint_name_to_id_["r_arm_grip"]   = 31;
  joint_name_to_id_["l_arm_grip"]   = 30;

  /* wheel */
  // result_["r_leg_wh_p"] = new robotis_framework::DynamixelState();
  // result_["l_leg_wh_l"] = new robotis_framework::DynamixelState();
  // joint_name_to_id_["r_leg_wh_p"]   = 41;
  // joint_name_to_id_["l_leg_wh_p"]   = 42;

  wheel_module_state_  = new WheelModuleState();
  joint_state_    = new WheelJointState();
}

WheelModule::~WheelModule()
{
  queue_thread_.join();
}

void WheelModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_       = boost::thread(boost::bind(&WheelModule::queueThread, this));

  ros::NodeHandle ros_node;

  /* publish topics */
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_  = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);
}

void WheelModule::parseWheelPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  double mov_time;
  mov_time = doc["mov_time"].as<double>();

  wheel_module_state_->mov_time_ = mov_time;

  // parse via-point number
  int via_num;
  via_num = doc["via_num"].as<int>();

  wheel_module_state_->via_num_ = via_num;

  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  wheel_module_state_->via_time_.resize(via_num, 1);
  for (int num = 0; num < via_num; num++)
    wheel_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

  // parse via-point pose
  wheel_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  wheel_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  wheel_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  wheel_module_state_->joint_via_pose_.fill(0.0);
  wheel_module_state_->joint_via_dpose_.fill(0.0);
  wheel_module_state_->joint_via_ddpose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator it = via_pose_node.begin(); it != via_pose_node.end(); ++it)
  {
    int id;
    std::vector<double> value;

    id = it->first.as<int>();
    value = it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num; num++)
      wheel_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int id;
    double value;

    id = it->first.as<int>();
    value = it->second.as<double>();

    wheel_module_state_->joint_wheel_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  wheel_module_state_->all_time_steps_ = int(wheel_module_state_->mov_time_ / wheel_module_state_->smp_time_) + 1;
  wheel_module_state_->calc_joint_tra_.resize(wheel_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void WheelModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */

  // for gui
  ros::Subscriber wheel_pose_msg_sub = ros_node.subscribe("/robotis/wheel/wheel_pose", 5, &WheelModule::wheelPoseMsgCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void WheelModule::wheelPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (wheel_module_state_->is_moving_ == false)
  {
    if (msg->data != "")
    {
      std::string pose_name = msg->data;
      // set module of all joints -> this module
      setCtrlModule(module_name_);

      // wait to change module and to get goal position for wheel
      while (enable_ == false || has_goal_joints_ == false)
        usleep(8 * 1000);

      // parse wheel pose
      std::string wheel_pose_path = ros::package::getPath("thormang3_wheel_module") + "/data/" + pose_name + ".yaml";
      parseWheelPoseData(wheel_pose_path);

      // generate trajectory
      tra_gene_tread_ = boost::thread(boost::bind(&WheelModule::wheelPoseTrajGenerateProc, this));
    }
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void WheelModule::wheelPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double wheel_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = wheel_module_state_->joint_wheel_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (wheel_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(wheel_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  wheel_module_state_->smp_time_, wheel_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = wheel_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = wheel_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = wheel_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(wheel_module_state_->via_num_, wheel_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value,
                                                               0.0, 0.0, wheel_module_state_->smp_time_,
                                                               wheel_module_state_->via_time_, wheel_module_state_->mov_time_);
    }

    wheel_module_state_->calc_joint_tra_.block(0, id, wheel_module_state_->all_time_steps_, 1) = tra;
  }

  wheel_module_state_->is_moving_ = true;
  wheel_module_state_->cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void WheelModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  setCtrlModule(module_name_);
  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  wheel_module_state_->mov_time_ = 5.0;
  wheel_module_state_->all_time_steps_ = int(wheel_module_state_->mov_time_ / wheel_module_state_->smp_time_) + 1;

  wheel_module_state_->calc_joint_tra_.resize(wheel_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  wheel_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double wheel_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = wheel_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] wheel_value : " << wheel_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(wheel_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                wheel_module_state_->smp_time_, wheel_module_state_->mov_time_);

    wheel_module_state_->calc_joint_tra_.block(0, id, wheel_module_state_->all_time_steps_, 1) = tra;
  }

  wheel_module_state_->is_moving_ = true;
  wheel_module_state_->cnt_ = 0;
  wheel_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

void WheelModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  setCtrlModule(module_name_);
  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  for (std::map<std::string, double>::iterator it = joint_angle_pose.begin(); it != joint_angle_pose.end(); it++)
  {
    std::string joint_name = it->first;
    double joint_angle_rad = it->second;

    std::map<std::string, int>::iterator joint_name_to_id_it = joint_name_to_id_.find(joint_name);
    if (joint_name_to_id_it != joint_name_to_id_.end())
    {
      target_pose.coeffRef(joint_name_to_id_it->second, 0) = joint_angle_rad;
    }
  }

  wheel_module_state_->joint_pose_ = target_pose;

  wheel_module_state_->mov_time_ = 5.0;
  wheel_module_state_->all_time_steps_ = int(wheel_module_state_->mov_time_ / wheel_module_state_->smp_time_) + 1;

  wheel_module_state_->calc_joint_tra_.resize(wheel_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double wheel_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = wheel_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] wheel_value : " << wheel_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(wheel_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                wheel_module_state_->smp_time_, wheel_module_state_->mov_time_);

    wheel_module_state_->calc_joint_tra_.block(0, id, wheel_module_state_->all_time_steps_, 1) = tra;
  }

  wheel_module_state_->is_moving_ = true;
  wheel_module_state_->cnt_ = 0;
  wheel_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

bool WheelModule::isRunning()
{
  return wheel_module_state_->is_moving_;
}

void WheelModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

  has_goal_joints_ = true;

  /* ----- send trajectory ----- */

  if (wheel_module_state_->is_moving_ == true)
  {
    if (wheel_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Wheel Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = wheel_module_state_->calc_joint_tra_(wheel_module_state_->cnt_, id);

    wheel_module_state_->cnt_++;
  }

  /*----- set joint data -----*/

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
  }

  /*---------- initialize count number ----------*/

  if ((wheel_module_state_->cnt_ >= wheel_module_state_->all_time_steps_) && (wheel_module_state_->is_moving_ == true))
  {
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Wheel Pose");
    publishDoneMsg("wheel_init");

    wheel_module_state_->is_moving_ = false;
    wheel_module_state_->cnt_ = 0;

    // set all joints -> none
    if (wheel_pose_only_ == true)
    {
      setCtrlModule("none");
      wheel_pose_only_ = false;
    }
  }
}

void WheelModule::stop()
{
  return;
}

void WheelModule::setCtrlModule(std::string module)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void WheelModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Wheel";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

void WheelModule::publishDoneMsg(const std::string done_msg)
{
  std_msgs::String movement_msg;
  movement_msg.data = done_msg;
  movement_done_pub_.publish(movement_msg);
}