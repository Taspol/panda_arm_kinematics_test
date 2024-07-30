#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_planner");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Define the planning group for the Panda arm
  static const std::string PLANNING_GROUP = "panda_arm";

  // Initialize the MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Configure tolerances
  move_group.setGoalJointTolerance(0.001);
  move_group.setGoalPositionTolerance(0.001);
  move_group.setGoalOrientationTolerance(0.001);

  // Define the target position
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0.4;

  // Set the target position for the end-effector
  move_group.setPoseTarget(target_pose);
  bool success = false;
  // Plan the motion
  for(int i = 0 ; i < 600 ; i++){
  	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  ROS_INFO_NAMED("panda_arm_planner", "Planning %s", success ? "SUCCEEDED" : "FAILED");

  // Execute the plan if planning was successful
//  if (success)
//  {
//    move_group.move();
//  }

  ros::shutdown();
  return 0;

}
