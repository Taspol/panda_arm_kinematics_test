cmake_minimum_required(VERSION 3.0.2)
project(move_group_interface)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/graph_visualize.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})

install(TARGETS planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
