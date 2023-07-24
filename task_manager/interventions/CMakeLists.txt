cmake_minimum_required(VERSION 3.5)
project(interventions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#################################################
#Create a variable for all source code files of the standard behavior tree nodes
file(GLOB BT_NODES_SOURCES src/behavior_tree/nodes/**.cpp)

#Find all the CMake files of the behavior tree code and the interventions
#####################
####BEHAVIOR TREE####
#####################
include (${CMAKE_CURRENT_SOURCE_DIR}/src/behavior_tree/CMakeLists.txt)

######################
#TEST INTERVENTION A##
######################
include(${CMAKE_CURRENT_SOURCE_DIR}/src/interventionA/CMakeLists.txt)

set(BTCPP_LIBRARY "/usr/local/lib/libbehaviortree_cpp.so")
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(intervention_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(MAIN_LIBS
  ${rclcpp_LIBRARIES}
  ${BTCPP_LIBRARY}
  )
set(MAIN_INCLUDE_DIRS
  include
  ${rclcpp_INCLUDE_DIRS}
)
set(MAIN_DEPS
  rclcpp
  rclcpp_action
  intervention_msgs
  std_srvs
  sensor_msgs
)

#################################################
add_executable(task_manager_node
  src/task_manager.cpp
  src/logger.cpp
)
target_link_libraries(task_manager_node
  ${MAIN_LIBS}
)
target_include_directories(task_manager_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${MAIN_INCLUDE_DIRS}
)
ament_target_dependencies(task_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${MAIN_DEPS}
)

#################################################
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    task_manager_node
  DESTINATION
    lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
