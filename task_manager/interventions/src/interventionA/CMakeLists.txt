find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(intervention_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(BTCPP_LIBRARY "/usr/local/lib/libbehaviortree_cpp.so")
#Find all behavior tree node source files
file(GLOB BT_NODE_SOURCES src/behavior_tree/nodes/**.cpp)
file(GLOB BT_INIT_SOURCES src/behavior_tree/nodes/init_nodes/**.cpp)
file(GLOB BT_FINAL_SOURCES src/behavior_tree/nodes/final_nodes/**.cpp)

set(INTERVENTIONA_DEPS
  rclcpp
  rclcpp_action
  intervention_msgs
  std_srvs
  sensor_msgs
)

set(INTERVENTIONA_INCLUDE_DIRS 
include
${rclcpp_INCLUDE_DIRS}
)

set(INTERVENTIONA_LIBS
${rclcpp_LIBRARIES}
${BTCPP_LIBRARY}
)

#########################################
add_executable(interventionA_bt
  src/interventionA/interventionA_tree.cpp
  src/logger.cpp
  ${BT_NODE_SOURCES}
  ${BT_INIT_SOURCES}
  ${BT_FINAL_SOURCES}
)
target_link_libraries(interventionA_bt
  ${INTERVENTIONA_LIBS}
)
target_include_directories(interventionA_bt
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  ${INTERVENTIONA_INCLUDE_DIRS}
)
ament_target_dependencies(interventionA_bt
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${INTERVENTIONA_DEPS}
)

install(
  TARGETS
    interventionA_bt
  DESTINATION
    lib/${PROJECT_NAME}
)