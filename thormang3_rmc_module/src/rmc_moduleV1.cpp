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
  // /* arm */
  // result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
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

  // /* arm */
  // joint_name_to_id_["r_arm_sh_p1"] = 1;
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
  // sub1_ = ros_node.subscribe("/tutorial_topic", 10, &RMCModuleTutorial::topicCallback, this);
  // ros::Subscriber pos_sub = ros_node.subscribe("/robotis/rmc/pos_msg", 5,
  //                                                       &RMCModule::topicCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/rmc/kinematics_pose_msg", 5,
                                                               &RMCModule::kinematicsPoseMsgCallback, this);
  ros::Subscriber goal_CoM_pose_msg_sub = ros_node.subscribe("/robotis/rmc/CoM_des_msg", 5,
                                                               &RMCModule::goalCoMPoseMsgCallback, this);
  r_foot_pub = ros_node.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/rmc/r_foot_msg", 0);
  l_foot_pub = ros_node.advertise<thormang3_manipulation_module_msgs::KinematicsPose>("/robotis/rmc/l_foot_msg", 0);
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

Eigen::MatrixXd RMCModule::pInv(Eigen::MatrixXd A)
{
  return A.transpose() * (A * A.transpose()).inverse();
}

bool RMCModule::calcRMC(Eigen::MatrixXd S,
                        Eigen::VectorXd P_ref,
                        Eigen::VectorXd L_ref,
                        std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref)
{
  bool limit_success = false;
  bool vel_limit_success = false;
  //現時刻における角度θ、ベースリンクの位置/姿勢 pb/Rbから行列Aを求める
  Eigen::MatrixXd I_tilde;//全重心回りの慣性行列
  I_tilde = robotis_->calcInertiaC();
  std::vector<Eigen::MatrixXd> M(4);
  std::vector<Eigen::MatrixXd> H(4);
  std::vector<Eigen::MatrixXd> H_(4);
  // std::vector<int> MH_id_from = {1,2,15,16};
  std::vector<int> MH_id_from = {1,27,15,16};
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

  std::vector<Eigen::MatrixXd> J(4);

  //Eigen::MatrixXd M_b;
  //Eigen::MatrixXd H_b;
  double m_tilde;//全質量
  m_tilde = robotis_->calcTotalMass(0);
  Eigen::MatrixXd c_tilde(3,1);//全重心位置
  c_tilde = robotis_->calcCenterOfMass(0);
  Eigen::VectorXd th; //現在角度
  Eigen::VectorXd p_b(3); //現在のベースリンクの位置
  // Eigen::VectorXd R_b(3); //現在のベースリンクの姿勢
  // p_b = robotis_->thormang3_link_data_[0]->position_;
  p_b = robotis_->thormang3_link_data_[44]->position_;
  Eigen::VectorXd r_B_c(3);//ボディから全重心への位置ベクトル
  r_B_c = c_tilde - p_b;
  std::vector<Eigen::VectorXd> r_B_i(4);//ボディから四肢先端への位置ベクトル
  //右手(r_B_i(0)),左手(r_B_i(1)),右足(r_B_i(2)),左足(r_B_i(3))

  Eigen::MatrixXd A_1(6,6);//Inertia matrix の計算に必要な行列
  // std::vector<Eigen::MatrixXd> A_2(4);//Inertia matrix の計算に必要な行列
  std::vector<Eigen::Matrix<double,6,6>> A_3(4);//Inertia matrix の計算に必要な行列
  A_1 <<
    m_tilde * Eigen::MatrixXd::Identity(3,3) , -m_tilde * robotis_->cross(r_B_c),
    Eigen::MatrixXd::Zero(3,3)               , I_tilde;

  //とりあえずFreeは無視している
  //目標全運動量P_ref、L_refおよび目標速度ξ_ref(i)を設定し、ベクトルyを求める
  Eigen::VectorXd y;
  Eigen::VectorXd y_1(6);//とりあえず
  y_1 <<
    P_ref,
    L_ref;

  for (int i=0; i<MH_id.size(); i++){
    idx_ = robotis_->findRoute(MH_id_from[i],MH_id_to[i]);
    // idx_ = robotis_->findRoute(MH_id[i]);
    idx = Eigen::Map<Eigen::VectorXi>(&idx_[0], idx_.size());
    int end_id = idx.size() - 1;
    M[i] = Eigen::MatrixXd::Zero(3,idx.size());
    H[i] = Eigen::MatrixXd::Zero(3,idx.size());
    H_[i] = Eigen::MatrixXd::Zero(3,idx.size());
    for (int j=0; j<idx.size(); j++){
      m_tilde_j = robotis_->calcTotalMass(idx(end_id - j));
      c_tilde_j = robotis_->calcCenterOfMass(idx(end_id - j));
      I_tilde_j = robotis_->calcInertiaC(idx(end_id - j));

      r_j = robotis_->thormang3_link_data_[idx(end_id - j)]->position_;
      m_j = robotis_->cross(robotis_->thormang3_link_data_[end_id - j]->joint_axis_) * (c_tilde_j - r_j) * m_tilde_j;
      // m_j = robotis_->cross(a) * (c_tilde_j - r_j) * m_tilde_j;
      h_j = robotis_->cross(c_tilde_j) * m_j + I_tilde_j * robotis_->thormang3_link_data_[end_id - j]->joint_axis_;
      M[i].col(end_id - j) = m_j;
      H_[i].col(end_id - j) = h_j;
    }
    H[i] = H_[i] - robotis_->cross(robotis_->calcCenterOfMass(0))*M[i];
    
    J[i] = robotis_->calcJacobian(robotis_->findRoute(MH_id_from[i],MH_id_to[i]));

    r_B_i[i] = robotis_->thormang3_link_data_[MH_id[i]]->position_ - p_b;
    Eigen::MatrixXd A_2(M[i].rows() * 2,M[i].cols());
    //Inertia matrix の計算に必要な行列
    A_2 <<
      M[i],
      H[i];
    A_3[i] <<
      Eigen::MatrixXd::Identity(3,3) , -robotis_->cross(r_B_i[i]),
      Eigen::MatrixXd::Zero(3,3)     , Eigen::MatrixXd::Identity(3,3);

    if(i!=1){
      A_1 += - A_2 * pInv(J[i]) * A_3[i];
      y_1 += - A_2 * pInv(J[i]) * guzai_i_ref[i];    
    }else{
      MH_free = A_2;
    }
  }
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_1.rows(),A_1.cols() + MH_free.cols());
  A << A_1 ,MH_free;
  // std::cout << "A_1" <<std::endl<< A_1 <<std::endl;
  // std::cout << "MH_free" <<std::endl<< MH_free <<std::endl;
  // std::cout << "A" <<std::endl<< A <<std::endl;
  A = S * A;

  y = S * y_1;
  //ベースリンク速度ξbおよび四肢以外の関節速度θdot_freeを計算する
  //Eigen::VectorXd dth_free;
  Eigen::VectorXd guzai_b_ref;//maybe 0
  guzai_b_ref = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd dth_free_ref;//maybe 0
  dth_free_ref = Eigen::VectorXd::Zero(MH_free.cols());
  Eigen::VectorXd guzai_b_dth_free(guzai_b_ref.size() + dth_free_ref.size());
  Eigen::VectorXd guzai_b_dth_free_ref(guzai_b_ref.size() + dth_free_ref.size());
  guzai_b_dth_free_ref << guzai_b_ref,dth_free_ref;
  //(guzai_b,dth_free) = pInv(A)*y + E - pInv(A)*A*[guzai_b_ref,dth_free_ref];
  guzai_b_dth_free = pInv(A)*y + (Eigen::MatrixXd::Identity(guzai_b_dth_free_ref.size(),guzai_b_dth_free_ref.size()) - pInv(A)*A) * guzai_b_dth_free_ref;
  //ベース速度guzai_bと目標四肢関節速度guzai_ref(i)より四肢関節速度dth(i)を計算する
  std::vector<Eigen::VectorXd> dth(4);
  for(int i=0; i<4; i++){
    if(i!=1){
        dth[i] = pInv(J[i]) * (guzai_i_ref[i] - A_3[i] * guzai_b_dth_free.head(6));
    }else{
      dth[i] = guzai_b_dth_free.tail(guzai_b_dth_free.size()-6);
    }
    idx_ = robotis_->findRoute(MH_id_from[i],MH_id_to[i]);
    idx = Eigen::Map<Eigen::VectorXi>(&idx_[0], idx_.size());

    for(int j=0; j<idx.size(); j++){
      if ( dth[i](j) >= 5.0 ){//たぶんMAX6.5。とりあえず5にしておく
        vel_limit_success = false;
        break;
      }else{
        vel_limit_success = true;
        robotis_->thormang3_link_data_[idx(j)]->joint_angle_ += dth[i](j) * 0.008;
        robotis_->thormang3_link_data_[idx(j)]->joint_velocity_ = dth[i](j);
        if ( robotis_->thormang3_link_data_[ idx(j) ]->joint_angle_ >= robotis_->thormang3_link_data_[ idx(j) ]->joint_limit_max_ ){
          limit_success = false;
          break;
        }else if ( robotis_->thormang3_link_data_[ idx(j) ]->joint_angle_ <= robotis_->thormang3_link_data_[ idx(j) ]->joint_limit_min_ ){
          limit_success = false;
          break;
        }else limit_success = true;
      }
    }
    if (limit_success == false || vel_limit_success == false) break;
  }

  if (limit_success == true && vel_limit_success == true)
    return true;
  else
    return false;
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
    CoM_init = robotis_->calcCenterOfMass(0);
    std::cout << "initial CoM" << std::endl << CoM_init << std::endl;
  }

  robotis_->calcForwardKinematics4rmc(0);
  r_foot_msg.pose.position.x = robotis_->thormang3_link_data_[45]->position_(0);
  r_foot_msg.pose.position.y = robotis_->thormang3_link_data_[45]->position_(1);
  r_foot_msg.pose.position.z = robotis_->thormang3_link_data_[45]->position_(2);
  l_foot_msg.pose.position.x = robotis_->thormang3_link_data_[46]->position_(0);
  l_foot_msg.pose.position.y = robotis_->thormang3_link_data_[46]->position_(1);
  l_foot_msg.pose.position.z = robotis_->thormang3_link_data_[46]->position_(2);
  CoM_msg.pose.position.x = robotis_->calcCenterOfMass(0)(0);
  CoM_msg.pose.position.y = robotis_->calcCenterOfMass(0)(1);
  CoM_msg.pose.position.z = robotis_->calcCenterOfMass(0)(2);
  r_foot_pub.publish(r_foot_msg);
  l_foot_pub.publish(l_foot_msg);
  CoM_pub.publish(CoM_msg);

  /* ----- send trajectory ----- */
  if (sub_flag == 1){
    // publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
    // ik_start_rotation_ = robotis_->thormang3_link_data_[ik_id_end_]->orientation_;
    /* ----- inverse kinematics ----- */
    // setInverseKinematics(cnt_, ik_start_rotation_);
    Eigen::MatrixXd S; //選択行列
    S = Eigen::MatrixXd::Identity(6,6);//今は単位行列

    Eigen::VectorXd P_ref = Eigen::VectorXd::Zero(3);//目標並進運動量
    Eigen::VectorXd L_ref = Eigen::VectorXd::Zero(3);//目標角運動量
    double Kp = 5;
    // std::cout << "base"  << std::endl << robotis_->thormang3_link_data_[0]->position_ << std::endl;
    // std::cout << "pelvis"  << std::endl << robotis_->thormang3_link_data_[44]->position_ << std::endl;
    // std::cout << "left foot"  << std::endl << robotis_->thormang3_link_data_[46]->position_ << std::endl;
    // std::cout << "CoM" << std::endl << robotis_->calcCenterOfMass(0) << std::endl;
    if(CoM_flag == 1){
      P_ref(0) = robotis_->calcTotalMass(0) * (Kp*(goal_CoM_pose_msg_.pose.position.x - robotis_->calcCenterOfMass(0)(0)) + 0);
      P_ref(1) = robotis_->calcTotalMass(0) * (Kp*(goal_CoM_pose_msg_.pose.position.y - robotis_->calcCenterOfMass(0)(1)) + 0);
      P_ref(2) = robotis_->calcTotalMass(0) * (Kp*(goal_CoM_pose_msg_.pose.position.z - robotis_->calcCenterOfMass(0)(2)) + 0);
    }else{
      P_ref(0) = robotis_->calcTotalMass(0) * (Kp*(CoM_init(0) - robotis_->calcCenterOfMass(0)(0)));
      P_ref(1) = robotis_->calcTotalMass(0) * (Kp*(CoM_init(1) - robotis_->calcCenterOfMass(0)(1)));
      P_ref(2) = robotis_->calcTotalMass(0) * (Kp*(CoM_init(2) - robotis_->calcCenterOfMass(0)(2)));
    }

    std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref(4);//目標四肢先端速度
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

    //matsushima
    goal_subed_msg.pose.position.x = goal_kinematics_pose_msg_.pose.position.x;
    goal_subed_msg.pose.position.y = goal_kinematics_pose_msg_.pose.position.y;
    goal_subed_msg.pose.position.z = goal_kinematics_pose_msg_.pose.position.z;
    goal_subed_msg.pose.orientation.x = goal_kinematics_pose_msg_.pose.orientation.x;
    goal_subed_msg.pose.orientation.x = goal_kinematics_pose_msg_.pose.orientation.x;
    goal_subed_msg.pose.orientation.x = goal_kinematics_pose_msg_.pose.orientation.x;
    goal_subed_pub.publish(goal_subed_msg);

    bool ik_success = calcRMC(S,P_ref,L_ref,guzai_i_ref);
    
    if (ik_success == true){
      for (int id = 1; id <= MAX_JOINT_ID; id++){
        goal_joint_position_(id) = robotis_->thormang3_link_data_[id]->joint_angle_;
        goal_joint_velocity_(id) = robotis_->thormang3_link_data_[id]->joint_velocity_;
      }
    }
    else{
      ROS_INFO("----- ik failed -----");
      ROS_INFO("[end] send trajectory");
    }    
    sub_flag = 2;
  }
  
  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
    result_[joint_name]->goal_velocity_ = goal_joint_velocity_(joint_name_to_id_[joint_name]);
    // result_[joint_name]->goal_position_ = des_pos.data;
    // result_[joint_name]->goal_velocity_ = des_pos.data;
    // result_[joint_name]->goal_torque_ = des_pos.data;
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
