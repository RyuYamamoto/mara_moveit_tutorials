
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.h>
#include <moveit_msgs/msg/collision_object.h>

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_interface_demo");

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface::Options options(PLANNING_GROUP, "robot_description", node);
  moveit::planning_interface::MoveGroupInterface move_group(options);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface(node);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  geometry_msgs::msg::Pose target_pose1;

  target_pose1.orientation.w = 0.5;
  target_pose1.orientation.x = -0.5;
  target_pose1.orientation.y = -0.5;
  target_pose1.orientation.z = -0.5;

  target_pose1.position.x = -0.039;
  target_pose1.position.y = -0.156;
  target_pose1.position.z = 1.5;

  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success;

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = -1.0;
  joint_group_positions[1] = 0.0;
  joint_group_positions[2] = 1.57;
  joint_group_positions[3] = 0.0;
  joint_group_positions[4] = 0.0;
  joint_group_positions[5] = 0.0;

  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}
