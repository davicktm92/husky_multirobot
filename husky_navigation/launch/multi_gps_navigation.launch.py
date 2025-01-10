# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    lifecycle_nodes = ['map_server']

    map_server_config_path = os.path.join(
    get_package_share_directory('husky_navigation'),
    'maps',
    'map.yaml'
    )
   
    husky_description_dir = os.path.join(get_package_share_directory('husky_gazebo'), 'launch')
    gps_localization_dir = os.path.join(get_package_share_directory('husky_navigation'), 'launch')
    bringup_dir = get_package_share_directory('nav2_bringup')

    params_dir = os.path.join(get_package_share_directory('husky_navigation'), 'params')
    nav2_params_file = os.path.join(params_dir, 'nav2_multirobot_params_1.yaml')


    configured_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key='',
            param_rewrites='',
            convert_types=True)
    
    namespace = LaunchConfiguration('namespace', default='robot1')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(husky_description_dir,'gazebo.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gps_localization_dir,'dual_ekf_navsat.launch.py')),
        ),

        GroupAction([
            PushRosNamespace(
                namespace=namespace
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir,"launch","navigation_launch.py")
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    "params_file": configured_params,
                    "autostart": autostart,
                    "namespace": namespace,
                    }.items()
            ),
                
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_server_config_path}],
        ),


    ])
