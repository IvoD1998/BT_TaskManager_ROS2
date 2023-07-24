# BT_TaskManager_ROS2
Task management system for robot interventions constructed with the Behavior Tree CPP framework in ROS2

# General
This system allows you to construct robotic behavior based on the Behavior Tree CPP framework (see: https://www.behaviortree.dev/)
This repository contains the following folder:
  * task_manager: Contains everything regarding the task manager itself, and the interventions constructed via the Behavior Tree framework. You can use these files as templates for your own interventions
    * intervention_msgs: Contains all messages, services and actions required for the communication between components. If a new robot behavior is programmed using a ROS action for instance, the action file should be added here
    * interventions: Contains the source code of the task manager and the behavior tree based actions, combined with their configuration files
  * staubli_tx2_60l_moveit_config: Contains a demo robot moveit config
  * staubli_tx2_60l_description: Contains all elements of the demo robot

The full explanation of how the system works can be found in (ADD PAPER). 
A short recap:
  * Standard blackboard parameters are added to each node to enable node-progress communication
  * Interventions are triggered via a service request, which puts the intervention in the buffer of the manager
  * Multiple requests can happen at the same time, and are ordered based on their completion and priority level
  * Communication between all components is guarded via a lifeline-watchdog system

# Dependencies
Both ROS2 (tested with Humble) and Behavior Tree CPP (tested with V4.3.1) should be installed for this repository to work.
However, it is recommended to perform a rosdep update before building the workspace

# How to install 
  1) Create a new workspace and place the folders of this repository inside its src folder
  2) Build the workspace from the root folder as explained in the ROS2 tutorials

# How to use
To run the example code, after successfully building the workspace (each of the following steps is executed in a new, sourced terminal window): 
  1) ros2 launch staubli_tx2_60l_moveit_config staubli_tx2_60l_planning_execution.launch.py
  2) ros2 launch interventions interventions.launch.py
  3) ros2 run interventions task_manager_node

Once all listed components are running, in a new terminal window, you can trigger the test intervention by calling:
  * ros2 service call /trigger_intervention intervention_msgs/srv/TriggerIntervention "{intervention: intervention_A, priority: 0}" (you can change the priority value as desired, higher is more important)

All the code running can be further customised via the included config files. In the config folder, you can find the behavior tree xml files, together with the node configuration files with all utilised ROS-parameters
These can be altered to change i.e. action names, update frequencies, timeout rates, ...
