"""Visualise the bipedal 5-bar model in RViz.

    ros2 launch assembly_description display.launch.py            # sliders
    ros2 launch assembly_description display.launch.py mode:=demo # animated
    ros2 launch assembly_description display.launch.py mode:=raw  # 8 free joints

The default path deliberately does NOT let joint_state_publisher_gui drive
/joint_states directly. Each leg is a closed 5-bar, so its knees are dependent
on its hips; driving all eight joints independently pulls the linkage apart at
the feet. Instead the GUI publishes onto "hip_states" and
five_bar_state_publisher solves the loop closure and republishes the full,
consistent state.

Use mode:=raw only when you specifically want to inspect the bare URDF tree.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('assembly_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'assembly.xacro')
    robot_urdf = xacro.process_file(xacro_file).toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument(
        name='mode',
        default_value='gui',
        description='gui (sliders + closure), demo (animated), or raw (8 free joints)',
    )

    is_gui = IfCondition(PythonExpression(["'", mode, "' == 'gui'"]))
    is_raw = IfCondition(PythonExpression(["'", mode, "' == 'raw'"]))
    is_closed = IfCondition(PythonExpression(["'", mode, "' in ['gui', 'demo']"]))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
    )

    # Sliders for all eight joints, but published onto hip_states; only the
    # four hip values are consumed downstream.
    joint_state_publisher_gui_node = Node(
        condition=is_gui,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('joint_states', 'hip_states')],
    )

    five_bar_node = Node(
        condition=is_closed,
        package='assembly_description',
        executable='five_bar_state_publisher',
        name='five_bar_state_publisher',
        parameters=[{'mode': mode}],
        output='screen',
    )

    # Escape hatch: drive the raw tree, loop closure ignored.
    raw_joint_state_publisher_gui_node = Node(
        condition=is_raw,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        mode_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        five_bar_node,
        raw_joint_state_publisher_gui_node,
        rviz_node,
    ])
