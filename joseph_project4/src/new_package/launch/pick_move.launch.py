from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    panda_pkg_dir = get_package_share_directory("sanchez_panda_config")

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="sanchez_panda_config")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    panda_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(panda_pkg_dir, "launch", "demo.launch.py")
        )
    )

    mtc_node = Node(
        package="new_package",
        executable="pick_move",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        panda_demo,
        TimerAction(period=5.0, actions=[mtc_node]),
    ])
