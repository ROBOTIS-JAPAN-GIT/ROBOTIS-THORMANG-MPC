#ifndef THORMANG3_RMC_MODULE_RMC_MODULE_H_
#define THORMANG3_RMC_MODULE_RMC_MODULE_H_

#include <map>
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

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

namespace thormang3
{

class RMCModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<RMCModule>
{
public:
  RMCModule();
  virtual ~RMCModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::Float64 &msg);
  void kinematicsPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg);
  void goalCoMPoseMsgCallback(const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg);

  /* Parameter */
  KinematicsDynamics *robotis_;

private:
  void queueThread();

  void parseData(const std::string &path);
  Eigen::MatrixXd pInv(Eigen::MatrixXd A);
  /* int calcRMC(); */
  /* bool calcRMC(std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref); */
  bool calcRMC(Eigen::MatrixXd S,
               Eigen::VectorXd P_ref,
               Eigen::VectorXd L_ref,
               std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref);
  /* bool calcRMC(std::vector<Eigen::Matrix<double,6,1>> guzai_i_ref); */
  
  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  /* boost::thread  *traj_generate_tread_; */

  /* inverse kinematics */
  bool  ik_solving_;
  int   ik_id_start_;
  int   ik_id_end_;

  thormang3_manipulation_module_msgs::KinematicsPose goal_kinematics_pose_msg_;
  thormang3_manipulation_module_msgs::KinematicsPose goal_CoM_pose_msg_;
  thormang3_manipulation_module_msgs::KinematicsPose r_foot_msg;
  thormang3_manipulation_module_msgs::KinematicsPose l_foot_msg;
  thormang3_manipulation_module_msgs::KinematicsPose CoM_msg;
  ros::Publisher r_foot_pub;
  ros::Publisher l_foot_pub;
  ros::Publisher CoM_pub;
  //matsushima
  thormang3_manipulation_module_msgs::KinematicsPose goal_subed_msg;
  ros::Publisher goal_subed_pub;
  Eigen::MatrixXd S; //選択行列
  float Kp,Ki,Kd;
  Eigen::Vector3d r_foot;
  Eigen::Vector3d l_foot;
  Eigen::Vector3d P_ref;//目標並進運動量
  Eigen::Vector3d L_ref;//目標角運動量
  double m_tilde;
  Eigen::Vector3d c_tilde;//重心位置
  double a_toe,b_toe;
  double a_heel,b_heel;
  double y_limit_max,y_limit_min;
  double x_limit_max,x_limit_min;
  double CoM_des_x,CoM_des_y,CoM_des_z;
  Eigen::Vector3d target;
  Eigen::Vector3d error;
  Eigen::Vector3d i_sum = Eigen::VectorXd::Zero(3);
  Eigen::Vector3d d_diff = Eigen::VectorXd::Zero(3);
  Eigen::Vector3d pre_target = Eigen::VectorXd::Zero(3);
  Eigen::Vector3d pre_curr = Eigen::VectorXd::Zero(3);
  /* Eigen::VectorXd P_ref = Eigen::VectorXd::Zero(3);//目標並進運動量 */
  /* Eigen::VectorXd L_ref = Eigen::VectorXd::Zero(3);//目標角運動量 */


  /* joint state */
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd init_joint_position_;
  Eigen::VectorXd goal_joint_velocity_;

  int   sub_flag = 0;
  int   CoM_flag = 0;
  Eigen::VectorXd CoM_init;
  Eigen::MatrixXd ik_target_position_;
  Eigen::MatrixXd ik_start_rotation_;
  Eigen::MatrixXd ik_target_rotation_;
  Eigen::MatrixXd ik_weight_;
  
  std::map<std::string, int> joint_name_to_id_;
};

}

#endif /* THORMANG3_RMC_MODULE_RMC_MODULE_H_ */
