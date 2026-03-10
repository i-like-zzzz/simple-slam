from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    system_mode = LaunchConfiguration("system_mode")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=get_package_share_directory("simple_slam") + "/config/simple_slam_2d.yaml",
            description="simple_slam 参数文件路径",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="是否使用仿真时间",
        ),
        DeclareLaunchArgument(
            "system_mode",
            default_value="mapping",
            description="运行模式: mapping | localization",
        ),
        Node(
            package="simple_slam",
            executable="simple_slam_node",
            name="simple_slam_node",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "system_mode": system_mode,
                },
            ],
        )
    ])
