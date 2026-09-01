# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Launch Gazebo Sim with command line arguments."""

import os
from os import environ
import shutil

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from ros2pkg.api import get_package_names
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import ExecuteProcess, Shutdown
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration


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
    if path.lower().endswith('.bat'):
        return os.path.splitext(path)[0]
    return path


def launch_gz(context, *args, **kwargs):
    model_paths, plugin_paths = GazeboRosPaths.get_paths()

    env = {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(
            [
                environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                environ.get("LD_LIBRARY_PATH", default=""),
                plugin_paths,
            ]
        ),
        "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
            [
                environ.get("GZ_SIM_RESOURCE_PATH", default=""),
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
    use_gui_value = LaunchConfiguration('use_gui').perform(context)

    # Decide GUI mode
    headless_flag = ""
    if use_gui_value in ["false", "False"]:
        headless_flag = " -s "

    if not len(gz_args) and len(ign_args):
        print("ign_args is deprecated, migrate to gz_args!")
        exec_args = ign_args + headless_flag
    else:
        exec_args = gz_args + headless_flag

    # Build the gz command
    if len(ign_version) or (ign_version == '' and int(gz_version) < 7):
        exec = 'ruby ' + get_executable_path('ign') + ' gazebo'
        if len(ign_version):
            gz_version = ign_version
    else:
        exec = 'ruby ' + get_executable_path('gz') + ' sim'

    if debugger != 'false':
        debug_prefix = 'x-terminal-emulator -e gdb -ex run --args'
    else:
        debug_prefix = None

    if on_exit_shutdown != 'false':
        on_exit = Shutdown()
    else:
        on_exit = None

    # Silence all output
    cmd_str = f"{exec} {exec_args} --force-version {gz_version} >/dev/null 2>&1",
    ld = [ExecuteProcess(
        cmd=[cmd_str],
        name='gazebo',
        output='screen',
        additional_env=env,
        shell=True,
        prefix=debug_prefix,
        on_exit=on_exit
    )]

    if debug_env == 'true':
        ld.insert(0, LogInfo(f"Launching gazebo with the environment variables: {env}"))

    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='',
                              description='Arguments to be passed to Gazebo Sim'),
        DeclareLaunchArgument('gz_version', default_value='8',
                              description='Gazebo Sim major version'),
        DeclareLaunchArgument('ign_args', default_value='',
                              description='Deprecated: Arguments to be passed to Ignition Gazebo'),
        DeclareLaunchArgument('ign_version', default_value='',
                              description='Deprecated'),
        DeclareLaunchArgument('debugger', default_value='false',
                              description='Run in Debugger'),
        DeclareLaunchArgument('debug_env', default_value='false',
                              description='Debug environment variables'),
        DeclareLaunchArgument('on_exit_shutdown', default_value='false',
                              description='Shutdown on exit'),
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='Launch Gazebo GUI (true) or headless (false)'),
        OpaqueFunction(function=launch_gz),
    ])
