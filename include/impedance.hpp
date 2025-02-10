#ifndef IMPEDACE_H
#define IMPEDACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/joint_command_interface.hpp>
#include <controller_interface/controller.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "KDL_Base.hpp"
#include <kdl/chainidsolver_recursive_newton_euler.hpp> // Inverse Dynamics
#include <kdl/chaindynparam.hpp>                        // Inverse Dynamics Params

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>
// ROS message includes - note the new ROS 2 paths and extensions:
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "joint_effort_msg/msg/JointEffort.hpp"
#include "joint_effort_msg/msg/JointEfforts.hpp"
#include "joint_state_msg/msg/JointState.hpp"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class Impedance : public kdl_base::KDL_Base
{
public:
  Impedance() {}
  ~Impedance() {}

  // Note: Instead of a ros::NodeHandle, we pass an rclcpp::Node shared pointer.
  void init( 
    rclcpp::Node::SharedPtr node,
    const std::string &topic_arm_state,
    const std::string &topic_arm_command,
    const std::string &topic_wrench_state,
    const std::vector<double> &Ka,
    const std::vector<double> &Kv,
    const std::vector<double> &Kp,
    const std::vector<double> &M,
    const std::vector<double> &D,
    const std::vector<double> &K,
    const std::vector<double> &desired_pose);

  void run();

private:
  void compute_impedance(bool flag);

  // In ROS 2, subscriptions pass messages as const SharedPtr
  void state_arm_callback(const joint_state_msg::msg::JointState::SharedPtr msg);
  void state_wrench_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void send_commands_to_robot();

protected:
  // Save our node pointer for timer-based or manual calls (e.g. using node_->now())
  rclcpp::Node::SharedPtr node_;

  // Subscribers:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
  rclcpp::Subscription<joint_state_msg::msg::JointState>::SharedPtr sub_arm_state_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_wrench_state_;
  // (If needed, add additional subscriptions such as for posture.)

  // Publisher:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_arm_cmd_;

  // Time and rate variables:
  rclcpp::Time last_publish_time_;
  double publish_rate_;

  // KDL Variables:
  KDL::JntArray Jnt_Pos_Init_State;
  KDL::JntArrayAcc Jnt_Desired_State;
  KDL::JntArray CMD_State;
  KDL::JntArray Current_State;
  KDL::JntArray Jnt_Toq_Cmd_;
  KDL::JntArray Jnt_Pos_State;
  KDL::JntArray Jnt_Vel_State;
  KDL::JntArray Jnt_Toq_State;

  KDL::Wrenches Ext_Wrenches;

  KDL::Rotation Desired_Ori_;
  KDL::Vector Desired_Pos_;
  KDL::Frame Desired_Pose_;

  KDL::Vector Gravity;

  KDL::JntSpaceInertiaMatrix M_; // Inertia Matrix
  KDL::JntArray C_, G_;
  KDL::JntArray Kp_, Kv_, Ka_;

  bool Recieved_Joint_State;
  bool Cmd_Flag_;
  bool Init_Flag_;
  unsigned int Step_;

  // Kinematics solvers:
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  boost::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;

  // Dynamics solvers:
  boost::shared_ptr<KDL::ChainIdSolver> id_pos_solver_;
  boost::shared_ptr<KDL::ChainDynParam> id_solver_;

  std::vector<double> Impedance_M, Impedance_D, Impedance_K;
  std::vector<double> desired_pose_;

  double wrench_x;
  double wrench_y;
  double wrench_z;
  double pos_x;
  double pos_y;
  double pos_z;
};

#endif  // IMPEDACE_H
