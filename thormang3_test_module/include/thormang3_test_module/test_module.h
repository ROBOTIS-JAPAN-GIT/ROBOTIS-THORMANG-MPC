#ifndef THORMANG3_TEST_MODULE_TEST_MODULE_H_
#define THORMANG3_TEST_MODULE_TEST_MODULE_H_

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

class TestModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<TestModule>
{
public:
  TestModule();
  virtual ~TestModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::Float64 &msg);
  
  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
  KinematicsDynamics *robotis_;

private:
  void queueThread();

  bool arm_angle_display_;

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::thread  *traj_generate_tread_;
  
  /* trajectory */
  bool    is_moving_;
  double  mov_time_;
  int     cnt_;
  int     all_time_steps_;


  /* inverse kinematics */
  bool  ik_solving_;
  int   ik_id_start_;
  int   ik_id_end_;

  std_msgs::Float64 des_pos;
  int sub_flag = 0;
  /* std_msgs::Float64 des_msg; */
  Eigen::VectorXd goal_joint_position_;

  std::map<std::string, int> joint_name_to_id_;
};

}

#endif /* THORMANG3_TEST_MODULE_TEST_MODULE_H_ */
