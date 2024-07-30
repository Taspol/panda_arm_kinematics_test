#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveGroupActionResult
import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from statistics import mean

csv_file = '/home/x390/Documents/Panda_arm_KDL.csv'
count = 0
total = 0
data2 = []
maxval = 0
maxjoint_pos = []
def result_callback(msg):
    global count 
    global total
    global data
    global maxval
    global maxjoint_pos
    joint_pos = [];
    deviation_all = 0
    planning_time = msg.result.planning_time
    joint_pos = msg.result.trajectory_start.joint_state.position
    rospy.loginfo("Planning time: %f seconds", planning_time)
    total = total + planning_time
    count = count + 1
    average = total / count 
    data2.append(planning_time)
    for ele in data2 :
    	deviation_all = deviation_all + abs(ele - average)
    
    data = {
       'joint_1': joint_pos[0],
       'joint_2': joint_pos[1],
       'joint_3': joint_pos[2],
       'planning_time': planning_time,
       'deviation' : deviation_all/count,
       'average' : average
    }
    if(planning_time > maxval) :
       maxval = planning_time
       maxjoint_pos = joint_pos
    
    df = pd.DataFrame([data])

    if os.path.exists(csv_file):
       # Append to the existing file without writing the header again
       df.to_csv(csv_file, mode='a', header=False, index=False)
    else:
       # Write to a new file with the header
       df.to_csv(csv_file, mode='w', header=True, index=False)

    rospy.loginfo("Data written to %s", csv_file)
    rospy.loginfo("average : %f ", average)
    rospy.loginfo("deviation_avg : %f ", deviation_all/count)
    plt.scatter(count, average, s=8, c='blue')
    plt.scatter(count, planning_time, s=8, c='lightgreen')
    i = 1
    rospy.loginfo("max_plnning_time  : %f" , maxval)

    for val in maxjoint_pos :  
        rospy.loginfo("joint %d : %f" , i , val)
        i = i + 1
    rospy.loginfo("------------------------")
    plt.pause(0.05)
def main():
    plt.axis([0, 1000 , 0.005, 0.06])

    rospy.init_node('move_group_result_subscriber')

    rospy.Subscriber('move_group/result', MoveGroupActionResult, result_callback)
    
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    main()

