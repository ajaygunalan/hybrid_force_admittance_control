/**
 * @file ur5e_loop_controller.cpp
 * @brief Control UR5e robot arm to perform repetitive movements between positions.
 *
 * This program creates a ROS 2 node that moves a UR5e arm between target and home positions.
 *
 * Action Client:
 *     /arm_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory):
 *         Commands for controlling arm joint positions
 *
 * Author: Addison Sears-Collins (modified)
 * Date: [Updated Date]
 */

 #include <chrono>
 #include <memory>
 #include <thread>
 #include <vector>
 
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp_action/rclcpp_action.hpp"
 #include "control_msgs/action/follow_joint_trajectory.hpp"
 #include "trajectory_msgs/msg/joint_trajectory_point.hpp"
 
 using namespace std::chrono_literals;
 
 class ArmLoopController : public rclcpp::Node {
 public:
     using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
 
     ArmLoopController() : Node("arm_loop_controller") {
         // Set up the action client for the arm controller
         arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, 
            "/joint_trajectory_controller/follow_joint_trajectory"
        );
 
         RCLCPP_INFO(this->get_logger(), "Waiting for arm action server...");
         if (!arm_client_->wait_for_action_server(20s)) {
             RCLCPP_ERROR(this->get_logger(), "Arm action server not available");
             return;
         }
         RCLCPP_INFO(this->get_logger(), "Arm action server connected!");
 
         // Initialize joint names and positions - Using UR5e joint names
         joint_names_ = {
             "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
         };
 
         // Define target positions
         target_pos_ = {0.0, -0.5, 0.5, -0.5, -0.5, 0.0};
         
         // Define home positions
         home_pos_ = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
 
         // Create a timer to control the robot movement cycle
         timer_ = this->create_wall_timer(10s, std::bind(&ArmLoopController::controlLoopCallback, this));
         
         // Start at home position
         sendArmCommand(home_pos_);
     }
 
 private:
     // Send a command to move the robot arm to specified joint positions.
     void sendArmCommand(const std::vector<double>& positions) {
         if (!arm_client_->wait_for_action_server(5s)) {
             RCLCPP_ERROR(this->get_logger(), "Action server not available");
             return;
         }
 
         // Create a goal message
         auto goal_msg = FollowJointTrajectory::Goal();
 
         // Set up the trajectory
         goal_msg.trajectory.joint_names = joint_names_;
         goal_msg.trajectory.points.resize(1);
         goal_msg.trajectory.points[0].positions = positions;
         goal_msg.trajectory.points[0].velocities.resize(positions.size(), 0.0);
         goal_msg.trajectory.points[0].accelerations.resize(positions.size(), 0.0);
         goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(2.0);
 
         RCLCPP_INFO(this->get_logger(), "Sending goal");
         
         // Send the goal
         auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
         send_goal_options.goal_response_callback =
             [this](auto) {
                 RCLCPP_INFO(this->get_logger(), "Goal accepted");
             };
         send_goal_options.feedback_callback =
             [this](auto, auto) {
                 // Do nothing with feedback for now
             };
         send_goal_options.result_callback =
             [this](const auto & result) {
                 RCLCPP_INFO(this->get_logger(), "Got result");
                 switch(result.code) {
                     case rclcpp_action::ResultCode::SUCCEEDED:
                         RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                         break;
                     case rclcpp_action::ResultCode::ABORTED:
                         RCLCPP_ERROR(this->get_logger(), "Goal aborted");
                         break;
                     case rclcpp_action::ResultCode::CANCELED:
                         RCLCPP_ERROR(this->get_logger(), "Goal canceled");
                         break;
                     default:
                         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                         break;
                 }
             };
 
         arm_client_->async_send_goal(goal_msg, send_goal_options);
     }
 
     // Execute one cycle of the control loop: move to target, wait 5 seconds,
     // then return to home and wait 5 seconds.
     void controlLoopCallback() {
         // Check if we're at home or target position
         static bool at_home = true;
         
         if (at_home) {
             RCLCPP_INFO(this->get_logger(), "Moving to target position");
             sendArmCommand(target_pos_);
         } else {
             RCLCPP_INFO(this->get_logger(), "Moving to home position");
             sendArmCommand(home_pos_);
         }
         
         // Toggle position for next callback
         at_home = !at_home;
     }
 
     // Action client and timer
     rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
     rclcpp::TimerBase::SharedPtr timer_;
 
     // Joint names and positions
     std::vector<std::string> joint_names_;
     std::vector<double> target_pos_;
     std::vector<double> home_pos_;
 };
 
 int main(int argc, char** argv) {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<ArmLoopController>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }