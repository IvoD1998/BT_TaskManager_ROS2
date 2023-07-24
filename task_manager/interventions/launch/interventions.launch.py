
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #Launch required components
    launch_files = []
    #Intervention A
    launch_files.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("interventions"), "launch/interventionA", "interventionA.launch.py")
            )
        )
    )
    #Add future interventions in the same way to have one global intervention launch file

    return LaunchDescription(
        launch_files 
    )
