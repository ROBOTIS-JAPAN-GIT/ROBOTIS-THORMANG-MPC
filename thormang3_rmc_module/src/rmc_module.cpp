/*
 *  rmc_module.cpp
 *
 *  Created on: 2019/06/25
 *      Author: Matsushima
 */

#include "thormang3_rmc_module/rmc_module.h"

using namespace thormang3;

RMCModule::RMCModule()
  : control_cycle_sec_(0.008)
{
  enable_       = false;
  module_name_  = "rmc_module";
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

  /* parameter */
  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  init_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  goal_joint_velocity_      = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  
  ik_id_start_  = 0;
  ik_id_end_    = 0;

  ik_target_position_   = Eigen::MatrixXd::Zero(3,1);
  ik_weight_            = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1, 1);
  ik_weight_.fill(1.0);
  
  robotis_                   = new KinematicsDynamics(WholeBody);
}

RMCModule::~RMCModule()
{
  queue_thread_.join();
}

void RMCModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  // control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&RMCModule::queueThread, this));

  std::string _path = ros::package::getPath("thormang3_rmc_module") + "/config/ik_weight.yaml";
  parseData(_path);
}

void RMCModule::parseData(const std::string &path)
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

void RMCModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/rmc/kinematics_pose_msg", 5,
                                                               &RMCModule::kinematicsPoseMsgCallback, this);
  ros::Subscriber goal_CoM_pose_msg_sub = ros_node.subscribe("/robotis/rmc/CoM_des_msg", 5,
                                                               &RMCModule::goalCoMPoseMsgCallback, this);
  // r_foot_pub = ros_node.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/rmc/r_foot_msg", 0);
  // l_foot_pub = ros_node.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/rmc/l_foot_msg", 0);
  CoM_pub = ros_node.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/rmc/CoM_msg", 0);

  //matsushima check
  goal_subed_pub = ros_node.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/rmc/goal_subed_msg", 0);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void RMCModule::goalCoMPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;
  goal_CoM_pose_msg_ = *msg;
  CoM_flag = 1;
  return;
}

void RMCModule::kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg)
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

void RMCModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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
    double joint_goal_velocity = dxl->dxl_state_->goal_velocity_;
    
    present_joint_position_(joint_name_to_id_[joint_name]) = joint_curr_position;
    goal_joint_position_(joint_name_to_id_[joint_name]) = joint_goal_position;
    goal_joint_velocity_(joint_name_to_id_[joint_name]) = joint_goal_velocity;
  }

  /*----- forward kinematics -----*/
  for (int id = 1; id <= MAX_JOINT_ID; id++){
    robotis_->thormang3_link_data_[id]->joint_angle_ = goal_joint_position_(id);
    robotis_->thormang3_link_data_[id]->joint_velocity_ = goal_joint_velocity_(id);
  }
  
  if (sub_flag == 0){
    robotis_->calcForwardKinematics(0);
    std::cout << "initial FK done"  << std::endl;
    sub_flag = 2;
    // CoM_init = robotis_->calcCenterOfMass(0);
    // robotis_->calcCenterOfMass();
    // robotis_->rmcInit();
    robotis_->rmcInit();
    CoM_init = robotis_->thormang3_link_data_[0]->c_tilde_;
    std::cout << "initial CoM" << std::endl << CoM_init << std::endl;
  }

  // robotis_->calcForwardKinematics4rmc(0);
  robotis_->rmcCalcParam();
  
  c_tilde = robotis_->thormang3_link_data_[0]->c_tilde_;
  m_tilde = robotis_->m_tilde;
  r_foot <<
    robotis_->thormang3_link_data_[45]->position_(0),
    robotis_->thormang3_link_data_[45]->position_(1),
    robotis_->thormang3_link_data_[45]->position_(2);
  l_foot <<
    robotis_->thormang3_link_data_[46]->position_(0),
    robotis_->thormang3_link_data_[46]->position_(1),
    robotis_->thormang3_link_data_[46]->position_(2);
  // std::cout << "l_foot" << l_foot << std::endl;
  CoM_msg.pose.position.x = c_tilde(0);
  CoM_msg.pose.position.y = c_tilde(1);
  CoM_msg.pose.position.z = c_tilde(2);
  CoM_pub.publish(CoM_msg);

  /* ----- send trajectory ----- */
  Kp = 5;
  Ki = 0.01;
  Kd = 0.02;
  if(CoM_flag == 1){
    //BoS とりあえず内側のみ
    //無理やりやっているけど本当は良くないと思う.暇なときに変更する.
    a_toe = ((l_foot(0) + 0.108) - (r_foot(0) + 0.108))
      / ((l_foot(1) - 0.057) - (r_foot(1) + 0.057));
    a_heel = ((l_foot(0) - 0.108) - (r_foot(0) - 0.108))
      / ((l_foot(1) - 0.057) - (r_foot(1) + 0.057));
    b_toe = (l_foot(0) + 0.108) - a_toe*(l_foot(1) - 0.057);
    b_heel = (l_foot(0) - 0.108) - a_heel*(l_foot(1) - 0.057);

    y_limit_max = l_foot(1) - 0.057;
    y_limit_min = r_foot(1) + 0.057;

    x_limit_max = a_toe * (c_tilde(1) + goal_CoM_pose_msg_.pose.position.y) + b_toe;
    x_limit_min = a_heel * (c_tilde(1) + goal_CoM_pose_msg_.pose.position.y) + b_heel;
    
    CoM_des_x = c_tilde(0) + goal_CoM_pose_msg_.pose.position.x;
    CoM_des_y = c_tilde(1) + goal_CoM_pose_msg_.pose.position.y;
    CoM_des_z = c_tilde(2) + goal_CoM_pose_msg_.pose.position.z;
    // CoM_des_x = CoM_init(0);
    // CoM_des_y = goal_CoM_pose_msg_.pose.position.y;
    // CoM_des_z = CoM_init(2);
    if(x_limit_max<=CoM_des_x) CoM_des_x = x_limit_max;
    else if(x_limit_min>=CoM_des_x) CoM_des_x = x_limit_min;
    if(y_limit_max<=CoM_des_y) CoM_des_y = y_limit_max;
    else if(y_limit_min>=CoM_des_y) CoM_des_y = y_limit_min;

    target <<
      CoM_des_x,
      CoM_des_y,
      CoM_des_z;
    error = target - c_tilde;
    P_ref(0) = m_tilde * (Kp*error(0) + Ki*i_sum(0) + Kd*d_diff(0));
    P_ref(1) = m_tilde * (Kp*error(1) + Ki*i_sum(1) + Kd*d_diff(1));
    P_ref(2) = m_tilde * (Kp*error(2) + Ki*i_sum(2) + Kd*d_diff(2));
    i_sum += error;
    d_diff = (target - pre_target) - (c_tilde - pre_curr);
    pre_target = target;
    pre_curr = c_tilde;
  }else if(CoM_flag == 2){
    target = pre_target;
    error = target - c_tilde;
    P_ref(0) = m_tilde * (Kp*error(0) + Ki*i_sum(0) + Kd*d_diff(0));
    P_ref(1) = m_tilde * (Kp*error(1) + Ki*i_sum(1) + Kd*d_diff(1));
    P_ref(2) = m_tilde * (Kp*error(2) + Ki*i_sum(2) + Kd*d_diff(2));
    i_sum += error;
    d_diff = (target - pre_target) - (c_tilde - pre_curr);
    pre_target = target;
    pre_curr = c_tilde;
  }else{
    target = CoM_init;
    error = target - c_tilde;
    P_ref(0) = m_tilde * (Kp*error(0));
    P_ref(1) = m_tilde * (Kp*error(1));
    P_ref(2) = m_tilde * (Kp*error(2));
    // P_ref(0) = m_tilde * (Kp*error(0) + Ki*i_sum(0) + Kd*d_diff(0));
    // P_ref(1) = m_tilde * (Kp*error(1) + Ki*i_sum(1) + Kd*d_diff(1));
    // P_ref(2) = m_tilde * (Kp*error(2) + Ki*i_sum(2) + Kd*d_diff(2));
    i_sum += error;
    d_diff = (target - pre_target) - (c_tilde - pre_curr);
    pre_target = target;
    pre_curr = c_tilde;
  }

  if(CoM_flag == 1){
    CoM_flag = 2;
  }
  
  L_ref << 0,0,0;
  S = Eigen::MatrixXd::Identity(6,6);//今は単位行列
  
  std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref(4);//目標四肢先端速度
  if (sub_flag == 1){
    guzai_i_ref[0] <<
      goal_kinematics_pose_msg_.pose.position.x,
      goal_kinematics_pose_msg_.pose.position.y,
      goal_kinematics_pose_msg_.pose.position.z,
      goal_kinematics_pose_msg_.pose.orientation.x,
      goal_kinematics_pose_msg_.pose.orientation.y,
      goal_kinematics_pose_msg_.pose.orientation.z;
    guzai_i_ref[1] << 0,0,0,0,0,0;
    guzai_i_ref[2] << 0,0,0,0,0,0;
    guzai_i_ref[3] << 0,0,0,0,0,0;
  }else{
    guzai_i_ref[0] << 0,0,0,0,0,0;
    guzai_i_ref[1] << 0,0,0,0,0,0;
    guzai_i_ref[2] << 0,0,0,0,0,0;
    guzai_i_ref[3] << 0,0,0,0,0,0;
  }

  //matsushima for check
  goal_subed_msg.pose.position.x = goal_kinematics_pose_msg_.pose.position.x;
  goal_subed_msg.pose.position.y = goal_kinematics_pose_msg_.pose.position.y;
  goal_subed_msg.pose.position.z = goal_kinematics_pose_msg_.pose.position.z;
  goal_subed_msg.pose.orientation.x = goal_kinematics_pose_msg_.pose.orientation.x;
  goal_subed_msg.pose.orientation.y = goal_kinematics_pose_msg_.pose.orientation.y;
  goal_subed_msg.pose.orientation.z = goal_kinematics_pose_msg_.pose.orientation.z;
  goal_subed_pub.publish(goal_subed_msg);
    
  bool ik_success = robotis_->calcRMC(S,P_ref,L_ref,guzai_i_ref);
    
  if (ik_success == true){
    for (int id = 1; id <= MAX_JOINT_ID; id++){
      goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
      // goal_joint_velocity_(id) = robotis_->thormang3_link_data_[id]->joint_velocity_;
    }
  }
  else{
    ROS_INFO("----- ik failed -----");
    ROS_INFO("[end] send trajectory");
  }
  sub_flag = 2;
  

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
    // result_[joint_name]->goal_velocity_ = goal_joint_velocity_(joint_name_to_id_[joint_name]);
  }
}

void RMCModule::stop()
{
  return;
}

bool RMCModule::isRunning()
{
  return false;
}
