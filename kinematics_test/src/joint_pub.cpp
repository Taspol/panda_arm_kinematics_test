#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iostream>
int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_trajectory_publisher");
    ros::NodeHandle nh;

    // Publisher to the arm controller command topic
    ros::Publisher joint_trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/rrr_arm/arm_controller/command", 10);

    // Set loop rate
    ros::Rate loop_rate(10);
    double x,y,z;
    while (ros::ok()) {
        // Create a JointTrajectory message
	std::cin >> x >> y >> z;
        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.joint_names.push_back("arm_joint_1"); // Add your joint names here
        joint_trajectory.joint_names.push_back("arm_joint_2"); // Add your joint names here
        joint_trajectory.joint_names.push_back("arm_joint_3"); // Add your joint names here

        // Create a JointTrajectoryPoint message
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(x); // Desired position for joint1
        point.positions.push_back(y); // Desired position for joint2
        point.positions.push_back(z); // Desired position for joint3
        point.time_from_start = ros::Duration(1.0); // Time to reach the point

        // Add the point to the trajectory
        joint_trajectory.points.push_back(point);

        // Publish the trajectory message
        joint_trajectory_pub.publish(joint_trajectory);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

