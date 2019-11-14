cmake_minimum_required(VERSION 3.10.2)
project(move_group_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
 ${EIGEN3_INCLUDE_DIRS})

include_directories(${rclcpp_INCLUDE_DIRS}
 ${moveit_ros_planning_interface_INCLUDE_DIRS})

link_directories(${rclcpp_LIBRARY_DIRS})

add_executable(move_group_interface src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface ${rclcpp_LIBRARIES} ${Boost_LIBRARIES} ${moveit_ros_planning_interface_LIBRARIES})
install(TARGETS move_group_interface DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(EIGEN3)

ament_package()
