/**
 * @file arm_loop_controller.cpp
 * @brief Control robot arm to perform repetitive movements between positions.
 *
 * This program creates a ROS 2 node that moves a robot arm between target and home positions.
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
             "/arm_controller/follow_joint_trajectory"
         );
 
         RCLCPP_INFO(this->get_logger(), "Waiting for arm action server...");
         if (!arm_client_->wait_for_action_server(20s)) {
             RCLCPP_ERROR(this->get_logger(), "Arm action server not available");
             return;
         }
         RCLCPP_INFO(this->get_logger(), "Arm action server connected!");
 
         // Initialize joint names and positions
         joint_names_ = {
             "link1_to_link2", "link2_to_link3", "link3_to_link4",
             "link4_to_link5", "link5_to_link6", "link6_to_link6_flange"
         };
 
         target_pos_ = {1.345, -1.23, 0.264, -0.296, 0.389, -1.5};
         home_pos_   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
 
         // Create timer for control loop
         timer_ = this->create_wall_timer(
             100ms,
             std::bind(&ArmLoopController::controlLoopCallback, this)
         );
     }
 
 private:
     // Send a command to move the robot arm to specified joint positions.
     void sendArmCommand(const std::vector<double>& positions) {
         auto goal_msg = FollowJointTrajectory::Goal();
         goal_msg.trajectory.joint_names = joint_names_;
 
         trajectory_msgs::msg::JointTrajectoryPoint point;
         point.positions = positions;
         point.time_from_start.sec = 2;  // 2 seconds for movement
         goal_msg.trajectory.points.push_back(point);
 
         auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
         arm_client_->async_send_goal(goal_msg, send_goal_options);
     }
 
     // Execute one cycle of the control loop: move to target, wait 5 seconds,
     // then return to home and wait 5 seconds.
     void controlLoopCallback() {
         // Move to target position and wait 5 seconds
         RCLCPP_INFO(this->get_logger(), "Moving to target position");
         sendArmCommand(target_pos_);
         std::this_thread::sleep_for(5000ms);  // 5 seconds wait
 
         // Return to home position and wait 5 seconds
         RCLCPP_INFO(this->get_logger(), "Returning to home position");
         sendArmCommand(home_pos_);
         std::this_thread::sleep_for(5000ms);  // 5 seconds wait
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
 
     try {
         rclcpp::spin(node);
     } catch (const std::exception& e) {
         RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
     }
 
     rclcpp::shutdown();
     return 0;
 }
 