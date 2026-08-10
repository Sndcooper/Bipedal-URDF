#!/usr/bin/env python3
"""Full simulation bring-up: Gazebo, the robot, controllers and the control stack.

Sequence
--------
1. Gazebo Harmonic starts with worlds/balance_arena.sdf
2. robot_state_publisher publishes the xacro-generated description
3. ros_gz_sim spawns the robot in the fallen pose
4. Controllers are spawned in dependency order
5. ros_gz_bridge maps /clock and /imu into ROS
6. balance_point, standing_mode, rc_teleop and wireless_gui_bridge start

The robot spawns lying down and does nothing until armed, matching the real
robot's start-up:

    ros2 service call /motors_enabled std_srvs/srv/SetBool "{data: true}"

Launch arguments
----------------
    spawn_pitch     Body pitch at spawn, radians. Negative is nose-up, i.e.
                    lying back. Default -1.35 (about -77 deg, flat on its
                    back). See the note on wheel authority below.
    spawn_z         Drop height. Default 0.12.
    use_closure_joints  Re-attach the 5-bar foot pivots as real SDF joints.
                    Default false; see assembly_gazebo.xacro for why.
    gui             Run the Gazebo GUI. Set false for headless batch runs.
    world           Path to an alternative world file.

WHEEL AUTHORITY NOTE
--------------------
The JGA12V motors are modelled at their 0.15 N.m stall torque, giving 0.30 N.m
across both wheels on a 2.269 kg robot - roughly 41 percent of body weight in
ground force at the 33 mm wheel radius. That is ample to balance around
upright, but it may not be enough to drive the body up from fully flat. If the
robot cannot complete the stand-up, raise spawn_pitch towards zero (say -0.8)
to start it partially reclined rather than assuming the gains are at fault:

    ros2 launch gazebo_prep simulation.launch.py spawn_pitch:=-0.8
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_prep')
    ros_ign_gazebo_share = get_package_share_directory('ros_ign_gazebo')

    default_world = os.path.join(pkg_share, 'worlds', 'balance_arena.sdf')
    controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    balance_params = os.path.join(pkg_share, 'config', 'balance_params.yaml')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='World file to load'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Run the Gazebo GUI (false for headless batch runs)'),
        DeclareLaunchArgument(
            'spawn_pitch', default_value='-1.35',
            description='Spawn body pitch in radians; negative lies back'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.12',
            description='Spawn height in metres'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0', description='Spawn X in metres'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0', description='Spawn Y in metres'),
        DeclareLaunchArgument(
            'use_closure_joints', default_value='false',
            description='Re-attach the 5-bar foot pivots as SDF joints '
                        '(experimental; conflicts with the mimic joints)'),
        DeclareLaunchArgument(
            'robot_name', default_value='assembly',
            description='Model name in Gazebo'),
    ]

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    use_closure_joints = LaunchConfiguration('use_closure_joints')
    robot_name = LaunchConfiguration('robot_name')

    # ------------------------------------------------------------------
    # Mesh resolution
    #
    # Gazebo resolves package:// by searching GZ_SIM_RESOURCE_PATH for a
    # directory whose name matches the package. Appending the PARENT of the
    # package share directory is what makes package://gazebo_prep/meshes/...
    # resolve.
    # ------------------------------------------------------------------
    resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.dirname(pkg_share),
            os.pathsep,
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ])

    # ------------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------------
    # With gui:=false, -s runs the server only. Passing it through ign_args is
    # the supported way to go headless; killing the GUI afterwards would race
    # against its startup.
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_ign_gazebo_share, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': ['-r -v 3 \'', world, '\''],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_ign_gazebo_share, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': ['-r -s -v 3 \'', world, '\''],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui),
    )

    # ------------------------------------------------------------------
    # Robot description
    # ------------------------------------------------------------------
    robot_description = ParameterValue(
        Command([
            'xacro \'',
            PathJoinSubstitution([
                FindPackageShare('gazebo_prep'), 'urdf', 'assembly.urdf.xacro']),
            '\' use_closure_joints:=', use_closure_joints,
            ' controllers_file:=\'', controllers_file, '\'',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ------------------------------------------------------------------
    # Spawn
    #
    # REP-103: positive pitch about +Y is nose-DOWN, so lying back is a
    # NEGATIVE pitch. -1.35 rad is about -77 degrees.
    # ------------------------------------------------------------------
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_assembly',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-R', '0.0',
            '-P', spawn_pitch,
            '-Y', '0.0',
        ],
    )

    # ------------------------------------------------------------------
    # Controllers
    #
    # Sequenced on process exit rather than started together: the broadcaster
    # must be up before the command controllers claim interfaces, and doing it
    # in order keeps the startup logs readable when something does go wrong.
    # ------------------------------------------------------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    active_controllers = Node(
        package='controller_manager',
        executable='spawner',
        name='active_controller_spawner',
        output='screen',
        arguments=[
            'hip_position_controller',
            'wheel_effort_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
    )

    # Loaded but not started; swap in for isolated wheel tests.
    inactive_controllers = Node(
        package='controller_manager',
        executable='spawner',
        name='inactive_controller_spawner',
        output='screen',
        arguments=[
            'wheel_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--inactive',
        ],
    )

    # ------------------------------------------------------------------
    # Gazebo <-> ROS bridge
    # ------------------------------------------------------------------
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # Control stack
    # ------------------------------------------------------------------
    balance_point = Node(
        package='gazebo_prep',
        executable='balance_point',
        name='balance_point',
        output='screen',
        parameters=[balance_params, {'use_sim_time': True}],
    )

    standing_mode = Node(
        package='gazebo_prep',
        executable='standing_mode',
        name='standing_mode',
        output='screen',
        parameters=[balance_params, {'use_sim_time': True}],
    )

    rc_teleop = Node(
        package='gazebo_prep',
        executable='rc_teleop',
        name='rc_teleop',
        output='screen',
        parameters=[balance_params, {'use_sim_time': True}],
    )

    wireless_gui_bridge = Node(
        package='gazebo_prep',
        executable='wireless_gui_bridge',
        name='wireless_gui_bridge',
        output='screen',
        parameters=[balance_params, {'use_sim_time': True}],
    )

    # ------------------------------------------------------------------
    # Ordering
    # ------------------------------------------------------------------
    after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[active_controllers],
        )
    )

    after_active = RegisterEventHandler(
        OnProcessExit(
            target_action=active_controllers,
            on_exit=[
                inactive_controllers,
                balance_point,
                standing_mode,
                rc_teleop,
                wireless_gui_bridge,
            ],
        )
    )

    return LaunchDescription(
        args + [
            resource_path,
            gz_sim_gui,
            gz_sim_headless,
            robot_state_publisher,
            spawn_robot,
            after_spawn,
            after_broadcaster,
            after_active,
            bridge,
        ]
    )
