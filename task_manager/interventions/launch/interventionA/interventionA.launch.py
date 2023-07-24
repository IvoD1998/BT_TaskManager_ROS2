
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
import xacro


parameter_file_path = Path(get_package_share_directory('interventions'), 'config/interventionA', 'interventionA_config.yaml')

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    #Load in robot description from description files
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("staubli_tx2_60l_moveit_config"),
            "config",
            "staubli_tx2_60l.urdf.xacro",
        )
    )
    #Load in all robot related components
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "staubli_tx2_60l_moveit_config", "config/staubli_tx2_60l.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = xacro.load_yaml(
        os.path.join(
            get_package_share_directory("staubli_tx2_60l_moveit_config"),
            "config",
            "kinematics.yaml",
        )
    )    
    nodes = []

    nodes.append(
        Node(
            package="interventions",
            executable="testA_action_server",
            output="screen",
            namespace="interventionA",
            parameters=[parameter_file_path],
        )
    )
    nodes.append(
        Node(
            package="interventions",
            executable="testB_action_server",
            output="screen",
            namespace="interventionA",
            parameters=[parameter_file_path],
        )
    )
    nodes.append(
        Node(
            package="interventions",
            executable="move_robot_tf_action_server",
            output="screen",
            namespace="interventionA",
            parameters=[parameter_file_path, robot_description, robot_description_semantic, kinematics_yaml],
        )
    )
    nodes.append(
        Node(
            package="interventions",
            executable="move_robot_named_action_server",
            output="screen",
            namespace="interventionA",
            parameters=[parameter_file_path, robot_description, robot_description_semantic, kinematics_yaml],
        )
    )
    nodes.append(
        Node(
            package="interventions",
            executable="move_robot_joint_state_action_server",
            output="screen",
            namespace="interventionA",
            parameters=[parameter_file_path, robot_description, robot_description_semantic, kinematics_yaml],
        )
    )

    nodes.append(
        Node(
            package="interventions",
            executable="interventionA_bt",
            output="screen",
            namespace="interventionA",
            parameters=[parameter_file_path],
        )
    )

    return LaunchDescription(
        nodes
        # launch_files 
    )
