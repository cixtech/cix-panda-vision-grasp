
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import importlib.util
import os
import sys
from pathlib import Path

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from cpu_affinity import taskset_prefix  # noqa: E402


def _affinity_enabled(context) -> bool:
    return LaunchConfiguration('use_cpu_affinity').perform(context).lower() in (
        'true', '1', 'yes', 'on',
    )


def _gazebo_cores(context) -> str:
    if not _affinity_enabled(context):
        return ''
    return LaunchConfiguration('gazebo_cpu_cores').perform(context)


def _rviz_cores(context) -> str:
    if not _affinity_enabled(context):
        return ''
    return LaunchConfiguration('rviz_cpu_cores').perform(context)


def _should_launch_moveit_py(context) -> bool:
    arg = LaunchConfiguration('launch_moveit_py').perform(context)
    if arg == 'auto':
        return importlib.util.find_spec('moveit') is not None
    return arg.lower() in ('true', '1', 'yes', 'on')


def _should_launch_grasp_bridge(context) -> bool:
    arg = LaunchConfiguration('launch_grasp_bridge').perform(context)
    return arg.lower() in ('true', '1', 'yes', 'on', 'auto')


def launch_setup(context, *args, **kwargs):
    packagepath = get_package_share_directory('panda_moveit_config')

    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'))

    arm_robot_sim_path = os.path.join(
        get_package_share_directory('panda_moveit_config'))

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(arm_robot_sim_path, 'worlds'), ':' +
            str(Path(robot_description_path).parent.resolve())
        ],
    )

    fastdds_xml = os.path.join(arm_robot_sim_path, 'config', 'fastdds_shm_only.xml')
    fastdds_profile = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_xml,
    )
    fastdds_profile_alt = SetEnvironmentVariable(
        name='FASTDDS_DEFAULT_PROFILES_FILE',
        value=fastdds_xml,
    )
    ros_localhost_only = SetEnvironmentVariable(
        name='ROS_LOCALHOST_ONLY',
        value='1',
    )
    gz_localhost = SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1')
    ign_localhost = SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1')

    moveit_config = (
        MoveItConfigsBuilder('panda_arm', package_name='panda_moveit_config')
        .robot_description('config/panda.urdf.xacro')
        .moveit_cpp(arm_robot_sim_path + '/config/controller_setting.yaml')
        .robot_description_semantic('config/panda.srdf')
        .to_moveit_configs()
    )

    world = LaunchConfiguration('world').perform(context)
    gazebo_cores = _gazebo_cores(context)
    rviz_cores = _rviz_cores(context)

    world_sdf = os.path.join(arm_robot_sim_path, 'worlds', f'{world}.sdf')
    gz_args = f'{world_sdf} -v 2 -r'

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(packagepath, 'launch', 'gz_sim_affinity.launch.py'),
        ),
        launch_arguments=[
            ('gz_args', gz_args),
            ('cpu_cores', gazebo_cores),
        ],
    )

    moveit_py_node = Node(
        name='commander',
        package='panda_moveit_config',
        executable='arm_control_from_UI.py',
        output='both',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    grasp_bridge_node = Node(
        name='grasp_candidate_bridge',
        package='panda_moveit_config',
        executable='grasp_candidate_bridge.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    robot_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', moveit_config.robot_description['robot_description'],
            '-x', '0.05',
            '-y', '0.0',
            '-z', '1.02',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', 'panda_arm',
            '-allow_renaming', 'false',
        ],
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    image_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
    )

    set_pose_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/arm_and_table/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
    )

    robot_desc_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': True},
            {'publish_frequency': 30.0},
        ],
    )

    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() in (
        'true', '1', 'yes', 'on',
    )

    rviz_prefix = taskset_prefix(rviz_cores)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        prefix=rviz_prefix,
        arguments=['-d', packagepath + '/config/moveit.rviz'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    controller_spawners = []
    for controller in [
        'joint_state_broadcaster',
        'panda_arm_controller',
        'panda_hand_controller',
    ]:
        controller_spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller,
                    '--controller-manager',
                    '/controller_manager',
                ],
                output='screen',
            ),
        )

    def _spawn_controllers_when_robot_ready(event, context):
        # Humble OnProcessExit.target_action must be ExecuteLocal, not Node.
        if event.action is not robot_to_gazebo_node or event.returncode != 0:
            return None
        return controller_spawners

    spawn_controllers_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(on_exit=_spawn_controllers_when_robot_ready),
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'world', '--child-frame-id', 'panda_link0'],
        parameters=[{'use_sim_time': True}],
    )

    actions = [
        gazebo_resource_path,
        fastdds_profile,
        fastdds_profile_alt,
        ros_localhost_only,
        gz_localhost,
        ign_localhost,
        LogInfo(msg=(
            f'[network] localhost only (ROS DDS + Gazebo): {fastdds_xml}'
        )),
        gazebo_node,
        robot_to_gazebo_node,
        spawn_controllers_after_robot,
        clock_bridge_node,
        robot_desc_node,
        move_group_node,
        image_bridge_node,
        set_pose_bridge_node,
        static_tf,
    ]

    if use_rviz:
        actions.insert(7, rviz_node)
    else:
        actions.append(
            LogInfo(msg='[gazebo.launch] RViz disabled (use_rviz:=false).'),
        )

    if _should_launch_moveit_py(context):
        actions.append(moveit_py_node)
        if _should_launch_grasp_bridge(context):
            actions.append(grasp_bridge_node)
    else:
        actions.append(
            LogInfo(msg=(
                '[gazebo.launch] Skipping arm_control_from_UI.py: Python package '
                "'moveit' not found. On ROS 2 Humble install moveit_py from source, "
                'or use launch_moveit_py:=true after installing.'
            )),
        )

    if _affinity_enabled(context):
        actions.insert(
            1,
            LogInfo(msg=(
                f'[cpu_affinity] Gazebo -> cores {gazebo_cores or "disabled"}, '
                f'RViz -> cores {rviz_cores or "disabled"}'
            )),
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='arm_on_the_table',
            description='Gazebo Sim world name (without .sdf)',
        ),
        DeclareLaunchArgument(
            'use_cpu_affinity',
            default_value='true',
            description='Bind Gazebo and RViz to dedicated CPU cores via taskset',
        ),
        DeclareLaunchArgument(
            'gazebo_cpu_cores',
            default_value='0,1,10,11',
            description='CPU cores for Gazebo Sim (big cores on CIX P1), e.g. "0" or "0-1,10-11"',
        ),
        DeclareLaunchArgument(
            'rviz_cpu_cores',
            default_value='9',
            description='CPU cores for RViz2, e.g. "9" or "4-5"',
        ),
        DeclareLaunchArgument(
            'launch_moveit_py',
            default_value='auto',
            description=(
                'Launch arm_control_from_UI.py (needs moveit Python bindings). '
                'auto: launch only if import moveit succeeds; true/false to force.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_grasp_bridge',
            default_value='true',
            description=(
                'Launch grasp_candidate_bridge.py (/grasp/candidates). '
                'Requires launch_moveit_py; ignored when commander is skipped.'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 (set false if GPU/MESA crashes the stack).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
