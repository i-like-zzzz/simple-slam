# Copyright 2026 zwc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    system_mode = LaunchConfiguration("system_mode")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    package_share = get_package_share_directory("simple_slam")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=package_share + "/config/simple_slam_2d.yaml",
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
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="是否同时启动 RViz",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=package_share + "/rviz/simple_slam.rviz",
            description="RViz 配置文件路径",
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
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(start_rviz),
        ),
    ])
