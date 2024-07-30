#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "set_joint_angles");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a publisher to the "rrr_arm/joint_states" topic
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("rrr_arm/joint_states", 10);

    // Set the loop rate
    ros::Rate loop_rate(2000);

    while (ros::ok())
    {
        // Create a JointState message
        sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
        // Set the joint names
        joint_state.name.push_back("arm_joint_1");
        joint_state.name.push_back("arm_joint_2");
        joint_state.name.push_back("arm_joint_3");

        // Set the joint positions (in radians)
        joint_state.position.push_back(1.0); // Desired angle for joint1
        joint_state.position.push_back(-0.8); // Desired angle for joint2
        joint_state.position.push_back(-1.3); // Desired angle for joint3

        // Optionally, you can also set velocities and efforts
//        joint_state.velocity.push_back(2.0); // Velocity for joint1
//        joint_state.velocity.push_back(2.0); // Velocity for joint2
//        joint_state.velocity.push_back(2.0); // Velocity for joint3
//
//        joint_state.effort.push_back(2.0); // Effort for joint1
//        joint_state.effort.push_back(5.0); // Effort for joint2
//        joint_state.effort.push_back(5.0); // Effort for joint3

        // Publish the message
        joint_pub.publish(joint_state);
	ROS_INFO("in the loop");
        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

