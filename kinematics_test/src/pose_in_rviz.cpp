#include <ros/ros.h>
#include <fstream>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
#include <unordered_map>
#include <string>
#include <iomanip>
#include <random>
#include <mutex>
#include <fstream>

std::mutex logMutex;

#define GREEN   "\033[32m"
#define YELLOW   "\x1b[33m"
// static const std::string PLANNING_GROUP = "arm";
const std::string PLANNING_GROUP = "panda_arm";
std::vector<std::pair<std::string,std::string>> test_log;

std::vector<double> joint_group_positions;


//-----check file-----
bool fileExists(std::string& fileName) {
    return static_cast<bool>(std::ifstream(fileName));
}

template <typename filename, typename T1, typename T2, typename T3>
bool writeCsvFile(filename &fileName, T1 column1, T2 column2, T3 column3) {
    std::lock_guard<std::mutex> csvLock(logMutex);
    std::fstream file;
    file.open (fileName, std::ios::out | std::ios::app);
    if (file) {
        file << column1 << ",";
        file << column2 << ",";
        file << column3 ;
        file <<  std::endl;
        return true;
    } else {
        return false;
    }
}


void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_group_positions = msg->position;
}


bool goToPoint(moveit::planning_interface::MoveGroupInterface &move_group, double x, double y, double z, std::string ee_frame){
  // Configure
  // move_group.setPlanningTime(60.0);
  move_group.setGoalJointTolerance(0.01);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.01);  
  
  // Set Goal
  ROS_INFO_STREAM("GOAL X: " << x << " Y: " << y << " Z: " << z);
  bool check = move_group.setPositionTarget(x, y, z, ee_frame);
  // ROS_INFO_STREAM("check" << check);
  move_group.setPlanningTime(5);
  // Planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ros::Time start_time = ros::Time::now();
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ros::Time end_time = ros::Time::now();
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ros::Duration solve_time = end_time - start_time;
  ROS_INFO_STREAM("Kinematic solve time: " << solve_time.toSec() << " seconds");
  // Moving
//  if(success){
//    moveit::core::MoveItErrorCode ret = move_group.execute(my_plan);
//    geometry_msgs::PoseStamped curr_pose = move_group.getCurrentPose(ee_frame);
//    ROS_INFO_STREAM(curr_pose);
//
//    // Print distance from goal to current pose
//    double distance = sqrt(pow(curr_pose.pose.position.x - x, 2) + pow(curr_pose.pose.position.y - y, 2) + pow(curr_pose.pose.position.z - z, 2));
//    ROS_INFO_STREAM("Distance: " << distance);
//
//    if(ret != moveit::core::MoveItErrorCode::SUCCESS){
//      ROS_ERROR_STREAM("Execution failed");
//      return false;
//    }
//  }
//  else{
//    //ROS_ERROR_STREAM("Planning failed");
//    return false;
//  }
  return success;
}

bool goToJointVal(moveit::planning_interface::MoveGroupInterface &move_group, double x, double y, double z, std::string ee_frame){
  // Configure
  // move_group.setPlanningTime(60.0);

  // Set Goal
  //move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joint_group_positions);
  //std::cout << "joint_1 : " << joint_group_positions[0] << "  joint_2 : " << joint_group_positions[1] << "  joint_3 : " << joint_group_positions[2] << std::endl;
    joint_group_positions[0] = x;  // Set joint 1 angle in radians
    joint_group_positions[1] = y; // Set joint 2 angle in radians
    joint_group_positions[2] = z;  // Set joint 3 angle in radians

  bool check = move_group.setJointValueTarget(joint_group_positions);

  // ROS_INFO_STREAM("check" << check);

  // Planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Moving
  //if(success){
  //  ROS_INFO_STREAM("before execute");
  //  moveit::core::MoveItErrorCode ret = move_group.execute(my_plan);
  //  ROS_INFO_STREAM("after execute");
  //  geometry_msgs::PoseStamped curr_pose = move_group.getCurrentPose(ee_frame);
  //  ROS_INFO_STREAM(curr_pose);

  //  if(ret != moveit::core::MoveItErrorCode::SUCCESS){
  //    ROS_ERROR_STREAM("Execution failed");

  //    return false;
  //  }
  //}
  //else{
  //  //ROS_ERROR_STREAM("Planning failed");
  //  return false;
  //}

  return success;
}

bool TEST(std::string name,double x , double y , double z , bool expect, moveit::planning_interface::MoveGroupInterface &move_group_interface,std::string type){
  ROS_INFO_STREAM("<----starting " << name <<"---->");
  bool is_success = false;
  if(type == "POSITION"){
     is_success = goToPoint(move_group_interface, x, y, z, "panda_hand");
  }else{
     is_success = goToJointVal(move_group_interface, x, y, z, "panda_hand");
  }
 
  if(is_success == expect){
  	  ROS_INFO_STREAM(GREEN << name  << " : PASS");
  	 // test_log.push_back(std::make_pair(name,"PASS"));
  }
  else{
  	  ROS_ERROR_STREAM(name << " : FAIL");
  	 // test_log.push_back(std::make_pair(name,"FAIL"));
  }
 return is_success;

}

void JointPub(double joint1, double joint2, double joint3, ros::Publisher &joint_pub) {
    // Create a JointState message
    sensor_msgs::JointState joint_state;

    // Set the timestamp
    joint_state.header.stamp = ros::Time::now();

    // Set the joint names
    joint_state.name = {
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        "panda_finger_joint1",
        "panda_finger_joint2"
    };

    // Set the joint positions (in radians)
    joint_state.position = {
        joint1,        // Position for panda_joint1
        joint2,        // Position for panda_joint2
        joint3,        // Position for panda_joint3
        -2.35619,      // Position for panda_joint4
        0.0,           // Position for panda_joint5
        1.5708,        // Position for panda_joint6
        0.785398,      // Position for panda_joint7
        0.035,         // Position for panda_finger_joint1
        0.035          // Position for panda_finger_joint2
    };

    // Publish the JointState message
    joint_pub.publish(joint_state);
}


struct test_data{
  double x;
  double y;
  double z;
  bool expect;
  std::string type;
};



int main(int argc, char** argv)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-2.5, 2.5);
  std::uniform_real_distribution<> dis2(-2, -3);

  double x,y,z;
  bool is_success = false;
  std::cout << "pose_test.cpp is running" << std::endl;
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback); 
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);  
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  

  // move_group_interface.getCurrentState()->copyJointGroupPositions(move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface.getName()), joint_group_positions);
    
  std::vector<std::pair<std::string,test_data>> alltest;

  ROS_INFO_STREAM("MOVE GROUP INTERFACE: " << move_group_interface.getName());
  ROS_INFO_STREAM("PLANNING FRAME: " << move_group_interface.getPlanningFrame());
  
  std::string command = "";
  std::cout << "Enter command : ";
  std::cin >> command;
  test_data data;
  std::string csvFile = "logfile_for_panda.csv";

  std::string t[] = {"7","18"};//input skip test
  std::set<std::string> skip_test;
  for(std::string s : t){
     skip_test.insert("TEST " + s); 
  }
  if(!fileExists(csvFile)){
     writeCsvFile(csvFile, "joint1", "joint2", "joint3");
  }
  int skip_count = 0, test_pass = 0,writeFailed = 0;
  bool test = true;
  std::vector<std::tuple<double,double,double>> available_angle , failedAngle;
  if(command == "0"){
 //   std::cin.ignore();
    //std::getline(std::cin,command);
    for(double i = -1.047 ; i <= 1.047 ; i+=0.25){
       for(double j = 0 ; j <= 0.733 ; j+=0.25){
          for(double k = -1.047; k <= 1.047 ; k+=0.25){

	         JointPub(k,j,i,joint_pub);
	         double random_y = dis(gen);
	         double random_x = dis2(gen);
             double j1_val = k;
	         double j2_val = 1.438;
	         double j3_val = -1.483;
             if(j > 0){
		            j2_val = -j2_val;
	         }
             if(!fileExists(csvFile)){
                //std::cout << "test1234456778" << std::endl;
                //writeCsvFile(csvFile, "joint1", "joint2", "joint3");
             }
             //if (!writeCsvFile(csvFile , k , j , i)) {
             //   std::cerr << "Failed to write to file: " << csvFile << "\n";
             //}  
 	         //if(j > -0.802 && j < 0){
             //       j1_val = -j1_val;
	         //}
             //if(k >= -1.57 && k < 1.57){
             //   k += 3.14;
             //}
             //if(j >= 0.000 && j < 0.733){
             //   j += 0.733;
             //}
             //if(i >= -1.047 && i < 1.047){
             //   i += 2.094;
             //}
            if (!writeCsvFile(csvFile,k,i,j)) {
              std::cerr << "Failed to write to file: " << csvFile <<
              writeFailed++;
            }
         
	         if(!TEST("TEST_ANGLE",-1.5,2,0.5,true,move_group_interface,"JointVal")){
                 failedAngle.push_back(std::make_tuple(k,j,i));
                if (!writeCsvFile(csvFile,k,i,j)) {
                  std::cerr << "Failed to write to file: " << csvFile << "\n";
                  writeFailed++;
                }
             }

             else test_pass++;
	         ROS_INFO_STREAM("Test_failed : " << failedAngle.size() << " Write failed : " << writeFailed );

	    }
       }
       available_angle.clear();
       //if(n.first == command) TEST(n.first,tmp.x,tmp.y,tmp.z,tmp.expect,move_group_interface,tmp.type);
    }
//    std::cout << "available angle : " << std::endl;
//    for(std::tuple<double,double,double> n : available_angle){
//      std::cout << "k : " << std::get<0>(n) << "j : " << std::get<1>(n) << "i : " << std::get<2>(n) << std::endl; 
//    } 
    std::cout << "failed angle : " << std::endl;
    for(std::tuple<double,double,double> n : failedAngle){
      std::cout << "joint1 : " << std::get<0>(n) << " joint2 : " << std::get<1>(n) << " joint3 : " << std::get<2>(n) << std::endl;
    }

  }else{
    for(std::pair<std::string,test_data> n : alltest){
       test_data tmp = n.second;
       if(skip_test.find(n.first) == skip_test.end()) TEST(n.first,tmp.x,tmp.y,tmp.z,tmp.expect,move_group_interface,tmp.type);
       else{
	  test_log.push_back(std::make_pair(n.first,"SKIP"));     
	  skip_count++;
       }
    }
  }
// writing to csv file
  if(!fileExists(csvFile)){
    writeCsvFile(csvFile, "joint1", "joint2", "joint3");
  }
  for(std::tuple<double,double,double> n : failedAngle){
    if (!writeCsvFile(csvFile,std::get<0>(n), std::get<1>(n), std::get<2>(n))) {
            std::cerr << "Failed to write to file: " << csvFile << "\n";
    }
  }


int test_fail = 0;
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("------- TEST REPORT -------");

 // for(std::pair<std::string,std::string> n : test_log){
 // 	if(n.second == "PASS"){
 //       	ROS_INFO_STREAM(GREEN << n.first  << " : PASS");
 //       	test_pass++;
 //       }else if(n.second == "FAIL"){
 //       	ROS_ERROR_STREAM(n.first << " : FAIL");
 //       	test_fail++;
 //       }else{
 //       	ROS_INFO_STREAM(YELLOW << n.first << " : SKIP");
 //       }
 // }
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("All test : " << test_pass + failedAngle.size());
  ROS_INFO_STREAM(GREEN << "TEST PASSED : " << test_pass);
  if(test_fail == 0) ROS_ERROR_STREAM( "TEST FAILED : " << failedAngle.size());
  if(skip_count != 0) ROS_INFO_STREAM(YELLOW << "TEST SKIPPED : " << skip_count);

  ros::shutdown();
  return 0;
}
