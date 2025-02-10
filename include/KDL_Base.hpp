#ifndef KDL_BASE_H
#define KDL_BASE_H

#include <urdf/model.h>
#include <controller_interface/controller.hpp>
#include <hardware_interface/joint_command_interface.hpp>

#include <rclcpp/rclcpp.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> // to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>

namespace kdl_base {

class KDL_Base
{
public:
  KDL_Base() {}
  virtual ~KDL_Base() {}

  // The init function now accepts an rclcpp::Node::SharedPtr rather than a ros::NodeHandle.
  bool init(const rclcpp::Node::SharedPtr & node);

protected:
  rclcpp::Node::SharedPtr node_;
  KDL::Chain kdl_chain_;
  KDL::JntArrayVel joint_state_;
  KDL::JntArray joint_effort_;

  struct limits_
  {
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
  } joint_limits_;
};

bool KDL_Base::init(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;

  // Get URDF and the names of the root and tip from the parameter server.
  std::string robot_description, root_name, tip_name;

  std::string name_space = node_->get_namespace();
  RCLCPP_INFO(node_->get_logger(), "--------------------> name_space: %s", name_space.c_str());

  // In ROS2 parameters must be declared beforehand.
  if (!node_->has_parameter("robot_description"))
  {
    RCLCPP_ERROR(node_->get_logger(), "KDL_Base: No robot description (URDF) found on parameter server (%s/robot_description)", name_space.c_str());
    return false;
  }
  node_->get_parameter("robot_description", robot_description);

  if (!node_->has_parameter("root_name"))
  {
    RCLCPP_ERROR(node_->get_logger(), "KDL_Base: No root name found on parameter server (%s/root_name)", name_space.c_str());
    return false;
  }
  node_->get_parameter("root_name", root_name);

  if (!node_->has_parameter("tip_name"))
  {
    RCLCPP_ERROR(node_->get_logger(), "KDL_Base: No tip name found on parameter server (%s/tip_name)", name_space.c_str());
    return false;
  }
  node_->get_parameter("tip_name", tip_name);

  // Use the robot_description string to build the URDF model.
  if (robot_description.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Parameter robot_description is empty, shutting down.");
    return false;
  }

  urdf::Model model;
  if (!model.initString(robot_description))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse urdf file");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Successfully parsed urdf file");

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree");
    return false;
  }

  // Populate the KDL chain from root to tip.
  if (!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get KDL chain from tree: %s --> %s", root_name.c_str(), tip_name.c_str());
    RCLCPP_ERROR(node_->get_logger(), "Tree has %d joints", kdl_tree.getNrOfJoints());
    RCLCPP_ERROR(node_->get_logger(), "Tree has %d segments", kdl_tree.getNrOfSegments());
    RCLCPP_ERROR(node_->get_logger(), "The segments are:");

    KDL::SegmentMap segment_map = kdl_tree.getSegments();
    for (const auto & seg : segment_map)
    {
      RCLCPP_ERROR(node_->get_logger(), "    %s", seg.first.c_str());
    }
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "tip_name: %s", tip_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "root_name: %s", root_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "Number of segments: %d", kdl_chain_.getNrOfSegments());
  RCLCPP_INFO(node_->get_logger(), "Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
  for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++) {
    RCLCPP_INFO(node_->get_logger(), "segment(%d): %s", static_cast<int>(i), kdl_chain_.getSegment(i).getName().c_str());
  }

  // Parse joint limits from the URDF model along the KDL chain.
  std::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
  std::shared_ptr<const urdf::Joint> joint_;
  joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
  joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
  joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
  int index;

  for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
  {
    joint_ = model.getJoint(link_->parent_joint->name);
    RCLCPP_INFO(node_->get_logger(), "Getting limits for joint: %s", joint_->name.c_str());
    index = kdl_chain_.getNrOfJoints() - i - 1;

    if (joint_->limits) {
      joint_limits_.min(index) = joint_->limits->lower;
      joint_limits_.max(index) = joint_->limits->upper;
      joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index)) / 2.0;
    } else {
      joint_limits_.min(index) = 0;
      joint_limits_.max(index) = 0;
      joint_limits_.center(index) = 0;
      RCLCPP_INFO(node_->get_logger(), "joint_->limits is NULL %s", joint_->name.c_str());
    }

    link_ = model.getLink(link_->getParent()->name);
  }

  RCLCPP_INFO(node_->get_logger(), "Finished Kinematic Base init");

  return true;
}

} // namespace kdl_base

#endif // KDL_BASE_H
