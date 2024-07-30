#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
#include <unordered_map>
#include <string>

#define GREEN   "\033[32m"
#define YELLOW   "\x1b[33m"
// static const std::string PLANNING_GROUP = "arm";
const std::string PLANNING_GROUP = "arm";
std::vector<std::pair<std::string,std::string>> test_log;
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

  // Planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Moving
  if(success){
    moveit::core::MoveItErrorCode ret = move_group.execute(my_plan);
    geometry_msgs::PoseStamped curr_pose = move_group.getCurrentPose(ee_frame);
    ROS_INFO_STREAM(curr_pose);

    // Print distance from goal to current pose
    double distance = sqrt(pow(curr_pose.pose.position.x - x, 2) + pow(curr_pose.pose.position.y - y, 2) + pow(curr_pose.pose.position.z - z, 2));
    ROS_INFO_STREAM("Distance: " << distance);

    if(ret != moveit::core::MoveItErrorCode::SUCCESS){
      ROS_ERROR_STREAM("Execution failed");
      return false;
    }
  }
  else{
    //ROS_ERROR_STREAM("Planning failed");
    return false;
  }
  return true;
}

bool goToJointVal(moveit::planning_interface::MoveGroupInterface &move_group, double x, double y, double z, std::string ee_frame){
  // Configure
  // move_group.setPlanningTime(60.0);

  // Set Goal
  std::vector<double> joint_group_positions;
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joint_group_positions);
  std::cout << "joint_1 : " << joint_group_positions[0] << "  joint_2 : " << joint_group_positions[1] << "  joint_3 : " << joint_group_positions[2] << std::endl;
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
  if(success){
    ROS_INFO_STREAM("before execute");
    moveit::core::MoveItErrorCode ret = move_group.execute(my_plan);
    ROS_INFO_STREAM("after execute");
    geometry_msgs::PoseStamped curr_pose = move_group.getCurrentPose(ee_frame);
    ROS_INFO_STREAM(curr_pose);

    if(ret != moveit::core::MoveItErrorCode::SUCCESS){
      ROS_ERROR_STREAM("Execution failed");

      return false;
    }
  }
  else{
    //ROS_ERROR_STREAM("Planning failed");
    return false;
  }

  return true;
}

void TEST(std::string name,double x , double y , double z , bool expect, moveit::planning_interface::MoveGroupInterface &move_group_interface,std::string type){
  ROS_INFO_STREAM("<----starting " << name <<"---->");
  bool is_success = false;
  if(type == "POSITION"){
     is_success = goToPoint(move_group_interface, x, y, z, "arm_4_link");
  }else{
     is_success = goToJointVal(move_group_interface, x, y, z, "arm_4_link");
  }

  if(is_success == expect){
	  ROS_INFO_STREAM(GREEN << name  << " : PASS");
  	  test_log.push_back(std::make_pair(name,"PASS"));
  }
  else{
	  ROS_ERROR_STREAM(name << " : FAIL");
	  test_log.push_back(std::make_pair(name,"FAIL"));
  }
 
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
  double x,y,z;
  bool is_success = false;
  std::cout << "pose_test.cpp is running" << std::endl;
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  

  std::vector<double> joint_group_positions;
  move_group_interface.getCurrentState()->copyJointGroupPositions(move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface.getName()), joint_group_positions);
    
  std::vector<std::pair<std::string,test_data>> alltest;

  ROS_INFO_STREAM("MOVE GROUP INTERFACE: " << move_group_interface.getName());
  ROS_INFO_STREAM("PLANNING FRAME: " << move_group_interface.getPlanningFrame());
  
  std::string command = "";
  std::cout << "Enter command : ";
  std::cin >> command;
  test_data data;

  std::string t[] = {"7","18"};//input skip test
  std::set<std::string> skip_test;
  for(std::string s : t){
     skip_test.insert("TEST " + s); 
  }
  //---------out of workspace---------
  if(command == "1" || command == "" || command == "0"){
     data.x = -2;  data.y = 2;  data.z = 4;  data.expect = false;  data.type = "POSITION";
     alltest.push_back(std::make_pair("TEST 1",data));
     
     data.x = -2;  data.y = -5;  data.z = 2;  data.expect = false;  data.type = "POSITION";
     alltest.push_back(std::make_pair("TEST 2",data));
     
     data.x = -4;  data.y = -2;  data.z = 2;  data.expect = false;  data.type = "POSITION";
     alltest.push_back(std::make_pair("TEST 3",data));
     
     data.x = 0;  data.y = -1;  data.z = 2;  data.expect = false;  data.type = "POSITION";
     alltest.push_back(std::make_pair("TEST 4",data));
  }

  //---------reach max of each joint constrain---------
  if(command == "2" || command == "" || command == "0"){
  data.x = 1.570;  data.y = -1.483;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 5",data));
 
  data.x = -1.570;  data.y = -1.483;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";  
  alltest.push_back(std::make_pair("TEST 6",data));

  data.x = -1.570;  data.y = 1.483;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 7",data));
  
  data.x = -1.570;  data.y = -1.483;  data.z = 1.1955080399300577;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 8",data));
  
  data.x = 1.570;  data.y = 1.483;  data.z = 1.1955080399300577;  data.expect = false;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 9",data));
  }

  //---------general move---------  
  if(command == "3" || command == "" || command == "0"){
  data.x = -3;  data.y = 0;  data.z = 2;  data.expect = true;  data.type = "POSITION";
  alltest.push_back(std::make_pair("TEST 10",data));  
  
  data.x = -2.5;  data.y = -2;  data.z = 0.1;  data.expect = true;  data.type = "POSITION";
  alltest.push_back(std::make_pair("TEST 11",data));
  
  data.x = -3;  data.y = 0;  data.z = 2;  data.expect = true;  data.type = "POSITION";
  alltest.push_back(std::make_pair("TEST 12",data));
  
  data.x = -2.5;  data.y = 2;  data.z = 0.1;  data.expect = true;  data.type = "POSITION";
  alltest.push_back(std::make_pair("TEST 13",data));
  

  }
 
//---------other weird config---------
  if(command == "4" || command == "" || command == "0"){
  data.x = 1.570;  data.y = -0.794;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 14",data));
  data.x = 1.570;  data.y = 0.752;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 15",data));
  data.x = -1.570;  data.y = -0.794;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 16",data));
  data.x = 1.570;  data.y = 0.752;  data.z = 1.1955080399300577;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 17",data));
  data.x = 1.570;  data.y = 1.483;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 18",data));
  data.x = -1.570;  data.y = 0.752;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
  alltest.push_back(std::make_pair("TEST 19",data));
  }


//---------Singularity test---------
  if(command == "5" || command == "" || command == "0"){
     data.x = 1.570;  data.y = -1.483;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
     alltest.push_back(std::make_pair("TEST 20",data));
     data.x = 1.570;  data.y = 0.752;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
     alltest.push_back(std::make_pair("TEST 21",data));
     data.x = -1.570;  data.y = -1.483;  data.z = 1.1955080399300577;  data.expect = true;  data.type = "JOINT";
     alltest.push_back(std::make_pair("TEST 22",data));
     data.x = 1.570;  data.y = 0.752;  data.z = 1.1955080399300577;  data.expect = true;  data.type = "JOINT";
     alltest.push_back(std::make_pair("TEST 23",data));
     data.x = 1.570;  data.y = 1.483;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
     alltest.push_back(std::make_pair("TEST 24",data));
     data.x = -1.570;  data.y = 0.752;  data.z = -1.0015643806514403;  data.expect = true;  data.type = "JOINT";
     alltest.push_back(std::make_pair("TEST 25",data));
    
  }
  int skip_count = 0;

  if(command == "0"){
    std::cin.ignore();
    std::getline(std::cin,command);
    for(std::pair<std::string,test_data> n : alltest){
       test_data tmp = n.second;
       if(n.first == command) TEST(n.first,tmp.x,tmp.y,tmp.z,tmp.expect,move_group_interface,tmp.type);
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


//---------out of workspace---------
//  TEST("TEST 1",-2,2,4,false,move_group_interface,"POSITION"); 
//  TEST("TEST 2",-2,-5,2,false,move_group_interface,"POSITION");  
//  TEST("TEST 3",-4,-2,2,false,move_group_interface,"POSITION"); 
//  TEST("TEST 4",0,-1,2,false,move_group_interface,"POSITION");

//---------reach max of each joint constrain---------
//  TEST("TEST 5", 1.571,-1.484,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 6", -1.571,-1.484,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 7", -1.571,1.484,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 8", -1.571,-1.484,1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 9", 1.571,1.484,1.484,true,move_group_interface,"JOINT");

//---------general move---------
//  TEST("TEST 10", -3,0,2,true,move_group_interface,"POSITION");
//  TEST("TEST 11", -2,2,1,true,move_group_interface,"POSITION");
//  TEST("TEST 12", -2.5,-2,0,true,move_group_interface,"POSITION");
//  TEST("TEST 13", -2.5,2,0,true,move_group_interface,"POSITION");

//---------other weird config---------
//  TEST("TEST 14", 1.571,-0.794,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 15", 1.571,0.752,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 16", -1.571,-0.794,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 17", 1.571,0.752,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 18", 1.571,1.484,-1.484,true,move_group_interface,"JOINT");
//  TEST("TEST 19", -1.571,0.752,-1.484,true,move_group_interface,"JOINT");


int test_pass = 0, test_fail = 0;
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("------- TEST REPORT -------");

  for(std::pair<std::string,std::string> n : test_log){
  	if(n.second == "PASS"){
		ROS_INFO_STREAM(GREEN << n.first  << " : PASS");
		test_pass++;
	}else if(n.second == "FAIL"){
		ROS_ERROR_STREAM(n.first << " : FAIL");
		test_fail++;
	}else{
		ROS_INFO_STREAM(YELLOW << n.first << " : SKIP");
	}
  }
  ROS_INFO_STREAM("---------------------------");
  ROS_INFO_STREAM("All test : " << test_pass + test_fail + skip_count);
  ROS_INFO_STREAM(GREEN << "TEST PASSED : " << test_pass);
  if(test_fail != 0) ROS_ERROR_STREAM( "TEST FAILED : " << test_fail);
  if(skip_count != 0) ROS_INFO_STREAM(YELLOW << "TEST SKIPPED : " << skip_count);

  ros::shutdown();
  return 0;
}
