#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
import os

def add_obstacle():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_obstacle_node')

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)

    obstacle_pose = geometry_msgs.msg.PoseStamped()
    obstacle_pose.header.frame_id = "base_link"  
    obstacle_pose.pose.position.x = -1.9
    obstacle_pose.pose.position.y = -0.73
    obstacle_pose.pose.position.z = 0.0
    obstacle_pose.pose.orientation.w = 1.0
    
    mesh_file = "/home/x390/newPalm/src/my_obstacle_pkg/meshes/Body1.stl"
    
    if not os.path.exists(mesh_file):
        rospy.logerr(f"Mesh file not found: {mesh_file}")
        return
    
    try:
        scene.add_mesh("obstacle", obstacle_pose, mesh_file)
    except Exception as e:
        rospy.logerr(f"Failed to add mesh: {e}")

    rospy.sleep(2)

if __name__ == '__main__':
    try:
        add_obstacle()
    except rospy.ROSInterruptException:
        pass

