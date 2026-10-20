# Copyright 2020 Open Source Robotics Foundation, Inc.
# Modified for CPU affinity (taskset) support in PandaVisionGrasp.
#
# Licensed under the Apache License, Version 2.0

"""Launch Gazebo Sim with optional CPU core binding via taskset."""

import os
import sys
from os import environ
import shutil

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from ros2pkg.api import get_package_names
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, Shutdown, LogInfo
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)
from cpu_affinity import merge_execute_prefix  # noqa: E402


class GazeboRosPaths:

    @staticmethod
    def get_paths():
        gazebo_model_path = []
        gazebo_plugin_path = []
        gazebo_media_path = []

        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)
            package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
            if os.path.isfile(package_file_path):
                try:
                    package = parse_package(package_file_path)
                except InvalidPackage:
                    continue
                for export in package.exports:
                    if export.tagname == 'gazebo_ros':
                        if 'gazebo_model_path' in export.attributes:
                            xml_path = export.attributes['gazebo_model_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_model_path.append(xml_path)
                        if 'plugin_path' in export.attributes:
                            xml_path = export.attributes['plugin_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_plugin_path.append(xml_path)
                        if 'gazebo_media_path' in export.attributes:
                            xml_path = export.attributes['gazebo_media_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_media_path.append(xml_path)

        gazebo_model_path = os.pathsep.join(gazebo_model_path + gazebo_media_path)
        gazebo_plugin_path = os.pathsep.join(gazebo_plugin_path)

        return gazebo_model_path, gazebo_plugin_path


def get_executable_path(command):
    path = shutil.which(command)
    if path is None:
        raise RuntimeError(f'Executable not found: {command}')
    if path.lower().endswith('.bat'):
        return os.path.splitext(path)[0]
    return path


def launch_gz(context, *args, **kwargs):
    model_paths, plugin_paths = GazeboRosPaths.get_paths()

    dds_env = {}
    for key in (
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE',
        'ROS_LOCALHOST_ONLY',
        'ROS_DOMAIN_ID',
    ):
        if key in environ and environ[key]:
            dds_env[key] = environ[key]

    env = {
        **dds_env,
        'GZ_IP': environ.get('GZ_IP', '127.0.0.1'),
        'IGN_IP': environ.get('IGN_IP', '127.0.0.1'),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            [
                environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                environ.get('LD_LIBRARY_PATH', default=''),
                plugin_paths,
            ]
        ),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            [
                environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                environ.get('LD_LIBRARY_PATH', default=''),
                plugin_paths,
            ]
        ),
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join(
            [
                environ.get('GZ_SIM_RESOURCE_PATH', default=''),
                model_paths,
            ]
        ),
        'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join(
            [
                environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''),
                model_paths,
            ]
        ),
    }

    gz_args = LaunchConfiguration('gz_args').perform(context)
    gz_version = LaunchConfiguration('gz_version').perform(context)
    ign_args = LaunchConfiguration('ign_args').perform(context)
    ign_version = LaunchConfiguration('ign_version').perform(context)
    debugger = LaunchConfiguration('debugger').perform(context)
    on_exit_shutdown = LaunchConfiguration('on_exit_shutdown').perform(context)
    debug_env = LaunchConfiguration('debug_env').perform(context)
    cpu_cores = LaunchConfiguration('cpu_cores').perform(context)

    if not len(gz_args) and len(ign_args):
        print('ign_args is deprecated, migrate to gz_args!')
        exec_args = ign_args
    else:
        exec_args = gz_args

    if len(ign_version) or (ign_version == '' and int(gz_version) < 7):
        exec_cmd = 'ruby ' + get_executable_path('ign') + ' gazebo'
        if len(ign_version):
            gz_version = ign_version
    else:
        exec_cmd = 'ruby ' + get_executable_path('gz') + ' sim'

    debug_prefix = 'x-terminal-emulator -e gdb -ex run --args' if debugger != 'false' else None
    prefix = merge_execute_prefix(cpu_cores, debug_prefix)

    on_exit = Shutdown() if on_exit_shutdown else None

    ld = [
        ExecuteProcess(
            cmd=[exec_cmd, exec_args, '--force-version', gz_version],
            output='screen',
            additional_env=env,
            shell=True,
            prefix=prefix,
            on_exit=on_exit,
        )
    ]
    if prefix and 'taskset' in prefix:
        ld.insert(0, LogInfo(msg=f'[cpu_affinity] Gazebo Sim bound to CPU cores: {cpu_cores}'))
    if debug_env == 'true':
        ld.insert(0, LogInfo(f'Launching gazebo with the environment variables: {env}'))

    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='', description='Arguments for Gazebo Sim'),
        DeclareLaunchArgument('gz_version', default_value='6', description='Gazebo Sim major version'),
        DeclareLaunchArgument('ign_args', default_value='', description='Deprecated: use gz_args'),
        DeclareLaunchArgument('ign_version', default_value='', description='Deprecated'),
        DeclareLaunchArgument('debugger', default_value='false', description='Run in debugger'),
        DeclareLaunchArgument('debug_env', default_value='false', description='Log env vars'),
        DeclareLaunchArgument('on_exit_shutdown', default_value='false', description='Shutdown on exit'),
        DeclareLaunchArgument(
            'cpu_cores',
            default_value='',
            description='CPU cores for taskset, e.g. "0-3". Empty disables binding.',
        ),
        OpaqueFunction(function=launch_gz),
    ])
