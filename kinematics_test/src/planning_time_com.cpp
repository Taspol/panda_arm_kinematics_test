#include <ros/ros.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <iostream>

double total = 0;
int count = 0;
void resultCallback(const moveit_msgs::MoveGroupActionResult::ConstPtr& msg)
{
    double planning_time = msg->result.planning_time;
    ROS_INFO("Planning time: %f seconds", planning_time);
    total+= planning_time;
    count++;
    double answer = total / count ;
    ROS_INFO("avg = %f " , answer);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_result_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber result_sub = nh.subscribe("move_group/result", 10, resultCallback);
    ros::spin();

    return 0;
}

