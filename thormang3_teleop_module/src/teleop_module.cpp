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
 *  manipulation_module.cpp
 *
 *  Created on: December 13, 2016
 *      Author: SCH
 */

#include "thormang3_teleop_module/teleop_module.h"

using namespace thormang3;

TeleopModule::TeleopModule()
  : control_cycle_sec_(0.008)
{
  enable_       = false;
  module_name_  = "teleop_module";
  control_mode_ = robotis_framework::PositionControl;
  // control_mode_ = robotis_framework::VelocityControl;
  // control_mode_ = robotis_framework::TorqueControl;
  
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
  result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"] = 1;
  joint_name_to_id_["l_arm_sh_p1"] = 2;
  joint_name_to_id_["r_arm_sh_r"]  = 3;
  joint_name_to_id_["l_arm_sh_r"]  = 4;
  joint_name_to_id_["r_arm_sh_p2"] = 5;
  joint_name_to_id_["l_arm_sh_p2"] = 6;
  joint_name_to_id_["r_arm_el_y"]  = 7;
  joint_name_to_id_["l_arm_el_y"]  = 8;
  joint_name_to_id_["r_arm_wr_r"]  = 9;
  joint_name_to_id_["l_arm_wr_r"]  = 10;
  joint_name_to_id_["r_arm_wr_y"]  = 11;
  joint_name_to_id_["l_arm_wr_y"]  = 12;
  joint_name_to_id_["r_arm_wr_p"]  = 13;
  joint_name_to_id_["l_arm_wr_p"]  = 14;
  joint_name_to_id_["torso_y"]     = 27;

  /* etc */
  joint_name_to_id_["r_arm_end"]   = 35;
  joint_name_to_id_["l_arm_end"]   = 34;

  /* parameter */
  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  init_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  ik_id_start_  = 0;
  ik_id_end_    = 0;

  ik_target_position_   = Eigen::MatrixXd::Zero(3,1);
  ik_weight_            = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1, 1);
  ik_weight_.fill(1.0);
  
  robotis_                   = new KinematicsDynamics(WholeBody);
}

TeleopModule::~TeleopModule()
{
  queue_thread_.join();
}

void TeleopModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  // control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&TeleopModule::queueThread, this));

  std::string _path = ros::package::getPath("thormang3_teleop_module") + "/config/ik_weight.yaml";
  parseData(_path);
}

void TeleopModule::parseData(const std::string &path)
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

  YAML::Node ik_weight_node = doc["weight_value"];
  for (YAML::iterator it = ik_weight_node.begin(); it != ik_weight_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    ik_weight_.coeffRef(id, 0) = value;
  }
}

void TeleopModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  // sub1_ = ros_node.subscribe("/tutorial_topic", 10, &TeleopModuleTutorial::topicCallback, this);
  // ros::Subscriber pos_sub = ros_node.subscribe("/robotis/teleop/pos_msg", 5,
  //                                                       &TeleopModule::topicCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/teleop/kinematics_pose_msg", 5,
                                                               &TeleopModule::kinematicsPoseMsgCallback, this);
  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

// void TeleopModule::topicCallback(const std_msgs::Float64 &msg)
// {
//   des_pos = msg;
//   sub_flag = 1;
//   // std::cout << "des pos" << des_pos.data <<std::endl;
// }

// void ManipulationModule::setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation)
// {
//   for (int dim = 0; dim < 3; dim++)
//     ik_target_position_.coeffRef(dim, 0) = goal_task_tra_.coeff(cnt, dim);

//   Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(start_rotation);

//   Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pose.orientation.w,
//                                        goal_kinematics_pose_msg_.pose.orientation.x,
//                                        goal_kinematics_pose_msg_.pose.orientation.y,
//                                        goal_kinematics_pose_msg_.pose.orientation.z);

//   double count = (double) cnt / (double) all_time_steps_;

//   Eigen::Quaterniond _quaternion = start_quaternion.slerp(count, target_quaternion);

//   ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(_quaternion);
// }

void TeleopModule::kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  goal_kinematics_pose_msg_ = *msg;

  if (goal_kinematics_pose_msg_.name == "left_arm")
  {
    ik_id_start_  = ID_L_ARM_START;
    ik_id_end_    = ID_L_ARM_END;
  }
  else if (goal_kinematics_pose_msg_.name == "right_arm")
  {
    ik_id_start_  = ID_R_ARM_START;
    ik_id_end_    = ID_R_ARM_END;
  }
  else if (goal_kinematics_pose_msg_.name == "left_arm_with_torso")
  {
    ik_id_start_  = ID_TORSO;
    ik_id_end_    = ID_L_ARM_END;
  }
  else if (goal_kinematics_pose_msg_.name == "right_arm_with_torso")
  {
    ik_id_start_  = ID_TORSO;
    ik_id_end_    = ID_R_ARM_END;
  }

  sub_flag = 1;
  
  return;
}


void TeleopModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                 std::map<std::string, double> sensors)
{
  if (enable_ == false){
    std::cout << "enable" <<std::endl;
    return;
  }
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

    present_joint_position_(joint_name_to_id_[joint_name]) = joint_curr_position;
    goal_joint_position_(joint_name_to_id_[joint_name]) = joint_goal_position;
  }

  /*----- forward kinematics -----*/
  for (int id = 1; id <= MAX_JOINT_ID; id++)
    robotis_->thormang3_link_data_[id]->joint_angle_ = goal_joint_position_(id);

  robotis_->calcForwardKinematics(0);

  /* ----- send trajectory ----- */
  if (sub_flag == 1){
    // publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
    // ik_start_rotation_ = robotis_->thormang3_link_data_[ik_id_end_]->orientation_;
    /* ----- inverse kinematics ----- */
    // setInverseKinematics(cnt_, ik_start_rotation_);
    ik_target_position_ <<
      goal_kinematics_pose_msg_.pose.position.x,
      goal_kinematics_pose_msg_.pose.position.y,
      goal_kinematics_pose_msg_.pose.position.z;
    Eigen::Quaterniond target_quaternion(goal_kinematics_pose_msg_.pose.orientation.w,
                                         goal_kinematics_pose_msg_.pose.orientation.x,
                                         goal_kinematics_pose_msg_.pose.orientation.y,
                                         goal_kinematics_pose_msg_.pose.orientation.z);
    ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(target_quaternion);

    int     max_iter    = 30;
    double  ik_tol      = 1e-3;
    bool    ik_success  = robotis_->calcInverseKinematics(ik_id_start_,
                                                          ik_id_end_,
                                                          ik_target_position_,
                                                          ik_target_rotation_,
                                                          max_iter, ik_tol,
                                                          ik_weight_);
    
    if (ik_success == true){
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
    }
    else{
      ROS_INFO("----- ik failed -----");
      ROS_INFO("[end] send trajectory");
    }
    
    sub_flag = 0;
  }
  
  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
    // result_[joint_name]->goal_position_ = des_pos.data;
    // result_[joint_name]->goal_velocity_ = des_pos.data;
    // result_[joint_name]->goal_torque_ = des_pos.data;
  }
}

void TeleopModule::stop()
{
  return;
}

bool TeleopModule::isRunning()
{
  return false;
}
