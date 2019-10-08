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

#include "thormang3_test_module/test_module.h"

using namespace thormang3;

TestModule::TestModule()
  : control_cycle_sec_(0.008),
    is_moving_(false),
    ik_solving_(false),
    arm_angle_display_(false)
{
  enable_       = false;
  module_name_  = "test_module";
  control_mode_ = robotis_framework::PositionControl;
  // control_mode_ = robotis_framework::VelocityControl;
  // control_mode_ = robotis_framework::TorqueControl;
  
  /* arm */
  result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  // result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  // result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
  // result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
  // result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  // result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  // result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
  // result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
  // result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
  // result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
  // result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
  // result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
  // result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();
  // result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();
  // result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"] = 1;
  // joint_name_to_id_["l_arm_sh_p1"] = 2;
  // joint_name_to_id_["r_arm_sh_r"]  = 3;
  // joint_name_to_id_["l_arm_sh_r"]  = 4;
  // joint_name_to_id_["r_arm_sh_p2"] = 5;
  // joint_name_to_id_["l_arm_sh_p2"] = 6;
  // joint_name_to_id_["r_arm_el_y"]  = 7;
  // joint_name_to_id_["l_arm_el_y"]  = 8;
  // joint_name_to_id_["r_arm_wr_r"]  = 9;
  // joint_name_to_id_["l_arm_wr_r"]  = 10;
  // joint_name_to_id_["r_arm_wr_y"]  = 11;
  // joint_name_to_id_["l_arm_wr_y"]  = 12;
  // joint_name_to_id_["r_arm_wr_p"]  = 13;
  // joint_name_to_id_["l_arm_wr_p"]  = 14;
  // joint_name_to_id_["torso_y"]     = 27;

  /* etc */
  // joint_name_to_id_["r_arm_end"]   = 35;
  // joint_name_to_id_["l_arm_end"]   = 34;

  /* parameter */
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);


  robotis_                   = new KinematicsDynamics(WholeBody);
}

TestModule::~TestModule()
{
  queue_thread_.join();
}

void TestModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  // control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&TestModule::queueThread, this));
}

void TestModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  // sub1_ = ros_node.subscribe("/tutorial_topic", 10, &TestModuleTutorial::topicCallback, this);
  ros::Subscriber pos_sub = ros_node.subscribe("/robotis/test/pos_msg", 5,
                                                        &TestModule::topicCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TestModule::topicCallback(const std_msgs::Float64 &msg)
{
  des_pos = msg;
  sub_flag = 1;
  // std::cout << "des pos" << des_pos.data <<std::endl;
}

void TestModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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

    // double joint_goal_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    goal_joint_position_(joint_name_to_id_[joint_name]) = joint_goal_position;
  }

  /* ----- send trajectory ----- */
  if (sub_flag == 1){
    for (int id = 1; id <= MAX_JOINT_ID; id++)
      goal_joint_position_(id) = des_pos.data;
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

void TestModule::stop()
{
  return;
}

bool TestModule::isRunning()
{
  return false;
}
