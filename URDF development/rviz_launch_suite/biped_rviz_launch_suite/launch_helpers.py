from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


@dataclass(frozen=True)
class RobotModelSpec:
    asset_package: str
    model_relpath: str
    model_kind: str  # 'urdf' or 'xacro'
    mesh_asset_package: str | None = None
    include_replacements: tuple[tuple[str, str], ...] = ()
    mesh_source_prefixes: tuple[str, ...] = ()
    mesh_final_prefixes: tuple[str, ...] = ()


def _package_share() -> Path:
    return Path(get_package_share_directory('biped_rviz_launch_suite'))


def _asset_dir(asset_package: str) -> Path:
    return _package_share() / 'assets' / asset_package


def _mesh_uri(asset_package: str) -> str:
    mesh_dir = _asset_dir(asset_package) / 'meshes'
    return mesh_dir.as_uri().rstrip('/') + '/'


def _apply_replacements(text: str, replacements: tuple[tuple[str, str], ...]) -> str:
    for old, new in replacements:
        text = text.replace(old, new)
    return text


def load_robot_description(spec: RobotModelSpec) -> str:
    asset_dir = _asset_dir(spec.asset_package)
    source_path = asset_dir / spec.model_relpath
    mesh_uri = _mesh_uri(spec.mesh_asset_package or spec.asset_package)

    source_text = source_path.read_text(encoding='utf-8')
    source_replacements = list(spec.include_replacements)
    source_replacements.extend((prefix, mesh_uri) for prefix in spec.mesh_source_prefixes)
    source_text = _apply_replacements(source_text, tuple(source_replacements))

    if spec.model_kind == 'xacro':
        with tempfile.NamedTemporaryFile(
            mode='w',
            suffix=source_path.suffix,
            dir=str(source_path.parent),
            delete=False,
            encoding='utf-8',
        ) as temp_file:
            temp_file.write(source_text)
            temp_path = Path(temp_file.name)

        try:
            robot_description = xacro.process_file(str(temp_path)).toxml()
        finally:
            try:
                temp_path.unlink()
            except FileNotFoundError:
                pass
    else:
        robot_description = source_text

    final_replacements = tuple((prefix, mesh_uri) for prefix in spec.mesh_final_prefixes)
    return _apply_replacements(robot_description, final_replacements)


def build_standard_display_launch_description(spec: RobotModelSpec) -> LaunchDescription:
    robot_description = load_robot_description(spec)
    rviz_config_file = _asset_dir(spec.asset_package) / 'config' / 'display.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='True')
    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config_file)],
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])


def build_assembly_display_launch_description(spec: RobotModelSpec) -> LaunchDescription:
    robot_description = load_robot_description(spec)
    rviz_config_file = _asset_dir(spec.asset_package) / 'config' / 'display.rviz'

    mode_arg = DeclareLaunchArgument(
        name='mode',
        default_value='gui',
        description='gui (sliders + closure), demo (animated), or raw (8 free joints)',
    )
    mode = LaunchConfiguration('mode')

    is_gui = IfCondition(PythonExpression(["'", mode, "' == 'gui'"]))
    is_raw = IfCondition(PythonExpression(["'", mode, "' == 'raw'"]))
    is_closed = IfCondition(PythonExpression(["'", mode, "' in ['gui', 'demo']"]))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        condition=is_gui,
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[('joint_states', 'hip_states')],
    )

    five_bar_node = Node(
        condition=is_closed,
        package='biped_rviz_launch_suite',
        executable='five_bar_state_publisher',
        name='five_bar_state_publisher',
        parameters=[{'mode': mode}],
        output='screen',
    )

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
        arguments=['-d', str(rviz_config_file)],
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