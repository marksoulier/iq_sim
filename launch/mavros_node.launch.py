from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pluginlists_yaml", description="Path to plugin lists YAML"
            ),
            DeclareLaunchArgument(
                "config_yaml", description="Path to configuration YAML"
            ),
            DeclareLaunchArgument("mavros_ns", description="MAVROS namespace"),
            DeclareLaunchArgument("fcu_url", description="FCU URL"),
            DeclareLaunchArgument("gcs_url", description="GCS URL"),
            DeclareLaunchArgument("respawn_mavros", description="Respawn MAVROS"),
            DeclareLaunchArgument("tgt_system", description="Target System ID"),
            DeclareLaunchArgument("tgt_component", description="Target Component ID"),
            DeclareLaunchArgument("log_output", description="Log Output"),
            Node(
                package="mavros",
                executable="mavros_node",
                namespace=LaunchConfiguration("mavros_ns"),
                output=LaunchConfiguration("log_output"),
                parameters=[
                    {
                        "fcu_url": LaunchConfiguration("fcu_url"),
                        "gcs_url": LaunchConfiguration("gcs_url"),
                        "target_system": LaunchConfiguration("tgt_system"),
                        "target_component": LaunchConfiguration("tgt_component"),
                        "respawn_mavros": LaunchConfiguration("respawn_mavros"),
                        "pluginlists_yaml": LaunchConfiguration("pluginlists_yaml"),
                        "config_yaml": LaunchConfiguration("config_yaml"),
                    }
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
