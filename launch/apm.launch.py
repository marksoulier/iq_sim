import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mavros_config_path = os.path.join(
        get_package_share_directory("mavros"), "launch", "apm_config.yaml"
    )
    mavros_pluginlists_path = os.path.join(
        get_package_share_directory("mavros"), "launch", "apm_pluginlists.yaml"
    )
    iq_sim_launch_path = os.path.join(
        get_package_share_directory("iq_sim"), "launch", "mavros_node.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value="udp://127.0.0.1:14551@14555",
                description="FCU URL",
            ),
            DeclareLaunchArgument("gcs_url", default_value=" ", description="GCS URL"),
            DeclareLaunchArgument(
                "tgt_system", default_value="1", description="Target System ID"
            ),
            DeclareLaunchArgument(
                "tgt_component", default_value="1", description="Target Component ID"
            ),
            DeclareLaunchArgument(
                "log_output", default_value="screen", description="Log Output"
            ),
            DeclareLaunchArgument(
                "respawn_mavros", default_value="true", description="Respawn MAVROS"
            ),
            DeclareLaunchArgument(
                "mavros_ns", default_value="/", description="MAVROS Namespace"
            ),
            DeclareLaunchArgument(
                "config_yaml",
                default_value=mavros_config_path,
                description="Configuration YAML file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(iq_sim_launch_path),
                launch_arguments={
                    "pluginlists_yaml": mavros_pluginlists_path,
                    "config_yaml": LaunchConfiguration("config_yaml"),
                    "mavros_ns": LaunchConfiguration("mavros_ns"),
                    "fcu_url": LaunchConfiguration("fcu_url"),
                    "gcs_url": LaunchConfiguration("gcs_url"),
                    "respawn_mavros": LaunchConfiguration("respawn_mavros"),
                    "tgt_system": LaunchConfiguration("tgt_system"),
                    "tgt_component": LaunchConfiguration("tgt_component"),
                    "log_output": LaunchConfiguration("log_output"),
                }.items(),
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
