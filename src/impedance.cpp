#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "joint_effort_msg/msg/joint_efforts.hpp"

#include "impedance/Impedance.h"      // Your header if needed
#include "impedance/KDL_Base.h"       // Adjust if needed
#include "kdl_conversions/kdl_msg.h"
#include <math.h>

class Impedance : public rclcpp::Node {
 public:
  // Constructor: no main function provided here
  Impedance(const std::string &node_name)
    : Node(node_name),
      Step_(0),
      Cmd_Flag_(true),
      Init_Flag_(true),
      Recieved_Joint_State(false),
      wrench_x(0), wrench_y(0), wrench_z(0),
      pos_x(0), pos_y(0), pos_z(0) {}

  // Conversion of the ROS1 init() method into ROS2.
  void init(std::string topic_arm_state,
            std::string topic_arm_command,
            std::string topic_wrench_state,
            std::vector<double> Ka,
            std::vector<double> Kv,
            std::vector<double> Kp,
            std::vector<double> M,
            std::vector<double> D,
            std::vector<double> K,
            std::vector<double> desired_pose) {
    // Subscribers
    sub_arm_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      topic_arm_state, 5,
      std::bind(&Impedance::state_arm_callback, this, std::placeholders::_1));

    sub_wrench_state_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      topic_wrench_state, 5,
      std::bind(&Impedance::state_wrench_callback, this, std::placeholders::_1));

    sub_posture_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joint_torque_controller/command", 1,
      std::bind(&Impedance::command, this, std::placeholders::_1));

    // Publisher
    pub_arm_cmd_ = this->create_publisher<joint_effort_msg::msg::JointEfforts>(
      topic_arm_command, 5);

    // KDL initialization via your base class (adjust shared pointer usage if needed)
    kdl_base::KDL_Base::init(*this);
    Gravity = KDL::Vector::Zero();
    Gravity(2) = -9.81;

    size_t nj = kdl_chain_.getNrOfJoints();
    Kp_.resize(nj);
    Kv_.resize(nj);
    Ka_.resize(nj);
    M_.resize(nj);
    C_.resize(nj);
    G_.resize(nj);

    // Kinematics solvers
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv_givens>(kdl_chain_);
    fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    fk_vel_solver_ = std::make_unique<KDL::ChainFkSolverVel_recursive>(kdl_chain_);
    ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(
      kdl_chain_, *(fk_pos_solver_.get()), *(ik_vel_solver_.get()));

    // Desired pose initialization (assumes desired_pose has 7 elements: x,y,z,qx,qy,qz,qw)
    desired_pose_ = desired_pose;
    Desired_Pos_ = KDL::Vector(desired_pose[0], desired_pose[1], desired_pose[2]);
    Desired_Ori_ = KDL::Rotation::Quaternion(
      desired_pose[3], desired_pose[4], desired_pose[5], desired_pose[6]);
    Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);

    // Dynamics solvers
    id_pos_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(kdl_chain_, Gravity);
    id_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, Gravity);

    // Get publish_rate parameter
    if (!this->get_parameter("publish_rate", publish_rate_)) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'publish_rate' not set");
    }

    // Initialize state variables
    Jnt_Pos_Init_State.resize(nj);
    Jnt_Desired_State.resize(nj);
    CMD_State.resize(nj);
    Current_State.resize(nj);
    Jnt_Pos_State.resize(nj);
    Jnt_Vel_State.resize(nj);
    Jnt_Toq_State.resize(nj);
    Jnt_Toq_Cmd_.resize(nj);

    Ext_Wrenches.resize(kdl_chain_.getNrOfSegments());
    KDL::Wrench wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));
    Ext_Wrenches.back() = wrench;

    for (size_t i = 0; i < nj; i++) {
      Kp_(i) = Kp[i];
      Kv_(i) = Kv[i];
      Ka_(i) = Ka[i];
    }

    Impedance_M = M;
    Impedance_D = D;
    Impedance_K = K;

    Recieved_Joint_State = false;
    Cmd_Flag_ = true;
    Init_Flag_ = true;
    Step_ = 0;
    wrench_z = 0;
    pos_z = 0;

    // Create a timer to run the control loop (200 Hz → 5ms period)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&Impedance::run, this));
  }

  // Callback for joint state messages
  void state_arm_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    size_t nj = kdl_chain_.getNrOfJoints();
    for (size_t i = 0; i < nj; i++) {
      Jnt_Pos_State(i) = msg->position[i];
      Jnt_Vel_State(i) = msg->velocity[i];
      Jnt_Toq_State(i) = msg->effort[i];
    }
    Recieved_Joint_State = true;
  }

  // Callback for wrench state messages
  void state_wrench_callback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    KDL::Wrench wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));
    Ext_Wrenches.back() = wrench;
    wrench_x = msg->force.x;
    wrench_y = msg->force.y;
    wrench_z = msg->force.z;
  }

  // Main impedance computation (invoked by the timer)
  void compute_impedance(bool flag) {
    if (flag) {
      if (Init_Flag_) {
        Jnt_Pos_Init_State = Jnt_Pos_State;
        Init_Flag_ = false;
      }
      if (wrench_z != 0) {
        KDL::Frame End_Pose;
        fk_pos_solver_->JntToCart(Jnt_Pos_State, End_Pose);

        KDL::JntArrayVel Jnt_Vel(Jnt_Pos_State, Jnt_Vel_State);
        KDL::FrameVel End_Pose_Vel;
        fk_vel_solver_->JntToCart(Jnt_Vel, End_Pose_Vel);

        KDL::Vector pose_p = End_Pose.p;
        KDL::Vector pose_vel_p = End_Pose_Vel.p.p;

        double acc_z = (wrench_z - (Impedance_D[2] * pose_vel_p(2) +
                        Impedance_K[2] * (desired_pose_[2] - pose_p(2)))) / Impedance_M[2];

        double dt = 0.005;  // timer period in seconds
        pos_z = 10 * (pose_vel_p(2) * dt + 0.5 * acc_z * dt * dt);

        // Update desired pose (x and y remain unchanged in this example)
        Desired_Pos_ = KDL::Vector(desired_pose_[0] + pos_x,
                                   desired_pose_[1] + pos_y,
                                   desired_pose_[2] + pos_z);
        Desired_Ori_ = KDL::Rotation::Quaternion(
          desired_pose_[3], desired_pose_[4], desired_pose_[5], desired_pose_[6]);
        Desired_Pose_ = KDL::Frame(Desired_Ori_, Desired_Pos_);
      }

      // Inverse kinematics to compute CMD_State from desired pose
      ik_pos_solver_->CartToJnt(Jnt_Pos_State, Desired_Pose_, CMD_State);

      double lambda = 0.1;
      double th = std::tanh(M_PI - lambda * Step_);
      double ch = std::cosh(M_PI - lambda * Step_);
      double sh2 = 1.0 / (ch * ch);

      size_t nj = kdl_chain_.getNrOfJoints();
      for (size_t i = 0; i < nj; i++) {
        Current_State(i) = CMD_State(i) - Jnt_Pos_Init_State(i);
        Jnt_Desired_State.q(i) = Current_State(i) * 0.5 * (1.0 - th) + Jnt_Pos_Init_State(i);
        Jnt_Desired_State.qdot(i) = Current_State(i) * 0.5 * lambda * sh2;
        Jnt_Desired_State.qdotdot(i) = Current_State(i) * lambda * lambda * sh2 * th;
      }
      Step_++;
      if (Jnt_Desired_State.q == CMD_State) {
        Cmd_Flag_ = false;
        Step_ = 0;
        Jnt_Pos_Init_State = Jnt_Pos_State;
      }

      // Compute dynamics: Inertia, Coriolis, and Gravity
      id_solver_->JntToMass(Jnt_Pos_State, M_);
      id_solver_->JntToCoriolis(Jnt_Pos_State, Jnt_Vel_State, C_);
      id_solver_->JntToGravity(Jnt_Pos_State, G_);

      // PID control law
      KDL::JntArray pid_cmd_(nj);
      KDL::JntArray cg_cmd_(nj);
      for (size_t i = 0; i < nj; i++) {
        pid_cmd_(i) = Ka_(i) * Jnt_Desired_State.qdotdot(i) +
                      Kv_(i) * (Jnt_Desired_State.qdot(i) - Jnt_Vel_State(i)) +
                      Kp_(i) * (Jnt_Desired_State.q(i) - Jnt_Pos_State(i));
        cg_cmd_(i) = C_(i) + G_(i);
      }
      Jnt_Toq_Cmd_.data = M_.data * pid_cmd_.data;
      KDL::Add(Jnt_Toq_Cmd_, cg_cmd_, Jnt_Toq_Cmd_);

      send_commands_to_robot();
      Recieved_Joint_State = false;
    }
  }

  // Callback for posture command messages
  void command(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    size_t nj = kdl_chain_.getNrOfJoints();
    if (msg->data.empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "Desired configuration must be of dimension %zu", nj);
    } else if (msg->data.size() != nj) {
      RCLCPP_ERROR(this->get_logger(),
                   "Posture message had the wrong size: %zu", msg->data.size());
      return;
    } else {
      for (size_t i = 0; i < nj; i++) {
        CMD_State(i) = msg->data[i];
      }
      RCLCPP_INFO(this->get_logger(), "Command SET");
      Cmd_Flag_ = true;
      Step_ = 0;
    }
  }

  // Publish joint torque commands
  void send_commands_to_robot() {
    auto msg = joint_effort_msg::msg::JointEfforts();
    msg.joint1_effort = Jnt_Toq_Cmd_(0);
    msg.joint2_effort = Jnt_Toq_Cmd_(1);
    msg.joint3_effort = Jnt_Toq_Cmd_(2);
    msg.joint4_effort = Jnt_Toq_Cmd_(3);
    msg.joint5_effort = Jnt_Toq_Cmd_(4);
    msg.joint6_effort = Jnt_Toq_Cmd_(5);
    pub_arm_cmd_->publish(msg);
  }

  // Control loop run by the timer
  void run() {
    RCLCPP_INFO(this->get_logger(), "Running the impedance control loop .................");
    compute_impedance(Recieved_Joint_State);
  }

 private:
  // ROS2 subscriptions, publisher, and timer
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_arm_state_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_wrench_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_posture_;
  rclcpp::Publisher<joint_effort_msg::msg::JointEfforts>::SharedPtr pub_arm_cmd_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // KDL variables (assumed to be set up via KDL_Base::init)
  KDL::Chain kdl_chain_;
  KDL::Vector Gravity;
  KDL::JntArray Kp_, Kv_, Ka_, M_, C_, G_;
  std::vector<double> desired_pose_, Impedance_M, Impedance_D, Impedance_K;
  KDL::Vector Desired_Pos_;
  KDL::Rotation Desired_Ori_;
  KDL::Frame Desired_Pose_;

  // KDL solvers
  std::unique_ptr<KDL::ChainIkSolverVel_pinv_givens> ik_vel_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;
  std::unique_ptr<KDL::ChainIdSolver_RNE> id_pos_solver_;
  std::unique_ptr<KDL::ChainDynParam> id_solver_;

  // Joint state arrays
  KDL::JntArray Jnt_Pos_Init_State, Jnt_Pos_State, Jnt_Vel_State, Jnt_Toq_State;
  KDL::JntArray CMD_State, Current_State, Jnt_Toq_Cmd_;
  KDL::JntArrayVel Jnt_Desired_State;  // Assumes members: q, qdot, and qdotdot

  std::vector<KDL::Wrench> Ext_Wrenches;
  double wrench_x, wrench_y, wrench_z;
  double pos_x = 0, pos_y = 0, pos_z;
  bool Recieved_Joint_State;
  bool Cmd_Flag_;
  bool Init_Flag_;
  unsigned int Step_;
  double publish_rate_;
};
