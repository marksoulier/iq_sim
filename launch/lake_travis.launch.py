import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the empty_world.launch.py in gazebo_ros package
    empty_world_launch_path = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "empty_world.launch.py"
    )

    # Get the path to the boat.world file in iq_sim package
    boat_world_path = os.path.join(
        get_package_share_directory("iq_sim"), "worlds", "lake_travis.world"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(empty_world_launch_path),
                launch_arguments={"world_name": boat_world_path}.items(),
            ),
            # more default parameters can be changed here
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
