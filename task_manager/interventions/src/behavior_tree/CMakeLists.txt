find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(intervention_msgs REQUIRED)

find_package(behaviortree_cpp REQUIRED)

set(BTCPP_LIBRARY "/usr/local/lib/libbehaviortree_cpp.so")

set(BEHAVIOR_TREE_DEPS
    rclcpp
    rclcpp_action
    tf2_ros
    moveit_ros_planning_interface
    sensor_msgs
    intervention_msgs
)
set(BEHAVIOR_TREE_LIBS
  ${rclcpp_LIBRARIES}
  ${BTCPP_LIBRARY}
)
set(BEHAVIOR_TREE_INCLUDE_DIRS
  include
  ${rclcpp_INCLUDE_DIRS}
)

#########################################
add_executable(testA_action_server
  src/behavior_tree/action_servers/testA_action_server.cpp
  src/logger.cpp
)
target_link_libraries(testA_action_server
  ${BEHAVIOR_TREE_LIBS}
)
target_include_directories(testA_action_server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${BEHAVIOR_TREE_INCLUDE_DIRS}
)
ament_target_dependencies(testA_action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${BEHAVIOR_TREE_DEPS}
)

#########################################
add_executable(testB_action_server
  src/behavior_tree/action_servers/testB_action_server.cpp
  src/logger.cpp
)
target_link_libraries(testB_action_server
  ${BEHAVIOR_TREE_LIBS}
)
target_include_directories(testB_action_server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${BEHAVIOR_TREE_INCLUDE_DIRS}
)
ament_target_dependencies(testB_action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${BEHAVIOR_TREE_DEPS}
)

#########################################
add_executable(move_robot_tf_action_server
  src/behavior_tree/action_servers/move_robot_tf_action_server.cpp
  src/logger.cpp
)
target_link_libraries(move_robot_tf_action_server
  ${BEHAVIOR_TREE_LIBS}
)
target_include_directories(move_robot_tf_action_server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${BEHAVIOR_TREE_INCLUDE_DIRS}
)
ament_target_dependencies(move_robot_tf_action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${BEHAVIOR_TREE_DEPS}
)

#########################################
add_executable(move_robot_named_action_server
  src/behavior_tree/action_servers/move_robot_named_action_server.cpp
  src/logger.cpp
)
target_link_libraries(move_robot_named_action_server
  ${BEHAVIOR_TREE_LIBS}
)
target_include_directories(move_robot_named_action_server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${BEHAVIOR_TREE_INCLUDE_DIRS}
)
ament_target_dependencies(move_robot_named_action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${BEHAVIOR_TREE_DEPS}
)

#########################################
add_executable(move_robot_joint_state_action_server
  src/behavior_tree/action_servers/move_robot_joint_state_action_server.cpp
  src/logger.cpp
)
target_link_libraries(move_robot_joint_state_action_server
  ${BEHAVIOR_TREE_LIBS}
)
target_include_directories(move_robot_joint_state_action_server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${BEHAVIOR_TREE_INCLUDE_DIRS}
)
ament_target_dependencies(move_robot_joint_state_action_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${BEHAVIOR_TREE_DEPS}
)

install(
  TARGETS
    testA_action_server
    testB_action_server
    move_robot_tf_action_server
    move_robot_named_action_server
    move_robot_joint_state_action_server
  DESTINATION lib/${PROJECT_NAME}
)