#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "sensor_msgs/JointState.h"
#include<iostream>
#include<cmath>

const std::string PLANNING_GROUP = "arm";

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "set_joint_angles");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);


    // Create a node handle
    ros::NodeHandle nh;

    // Create a publisher to the "rrr_arm/joint_states" topic
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("rrr_arm/joint_states", 10);

    // Set the loop rate
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
	std::vector<double> joint_group_positions;
        move_group_interface.getCurrentState()->copyJointGroupPositions(move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface.getName()), joint_group_positions);
        std::cout << "joint_1 : " << joint_group_positions[0] << "  joint_2 : " << joint_group_positions[1] << "  joint_3 : " << joint_group_positions[2] << std::endl;
	
        // Create a JointState message
//        sensor_msgs::JointState joint_state;
//	joint_state.header.stamp = ros::Time::now();
//        // Set the joint names
//        joint_state.name.push_back("arm_joint_1");
//        joint_state.name.push_back("arm_joint_2");
//        joint_state.name.push_back("arm_joint_3");
//
//        // Set the joint positions (in radians)
//        joint_state.position.push_back(2.0); // Desired angle for joint1
//        joint_state.position.push_back(2.5); // Desired angle for joint2
//        joint_state.position.push_back(0.0); // Desired angle for joint3
//
//        // Optionally, you can also set velocities and efforts
//        joint_state.velocity.push_back(2.0); // Velocity for joint1
//        joint_state.velocity.push_back(2.0); // Velocity for joint2
//        joint_state.velocity.push_back(2.0); // Velocity for joint3
//
//        joint_state.effort.push_back(2.0); // Effort for joint1
//        joint_state.effort.push_back(5.0); // Effort for joint2
//        joint_state.effort.push_back(5.0); // Effort for joint3
//
//        // Publish the message
//        joint_pub.publish(joint_state);
	ROS_INFO("in the loop");
        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

