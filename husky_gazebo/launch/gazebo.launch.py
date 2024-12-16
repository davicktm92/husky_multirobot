from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

import sys

import event_emitter as events

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    ld = LaunchDescription(ARGUMENTS)
    

    # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
    #         ),
    #         " ",
    #         "name:=husky",
    #         " ",<
    #         "prefix:=''",
    #         " ",
    #         "is_sim:=true",
    #         " ",
    #         "gazebo_controllers:=",
    #         config_husky_velocity_controller,
    #     ]
    # )
    # robot_description = {"robot_description": robot_description_content}

    # spawn_husky_velocity_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['husky_velocity_controller', '-c', '/controller_manager'],
    #     output='screen',
    # )

    # node_robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[{'use_sim_time': True}, robot_description,],
    # )

    # spawn_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    #     output='screen',
    # )

    # # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    # diffdrive_controller_spawn_callback = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_joint_state_broadcaster,
    #         on_exit=[spawn_husky_velocity_controller],
    #     )
    # )
    # # Gazebo server
    # gzserver = ExecuteProcess(
    #     cmd=['gzserver',
    #          '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so',
    #          world_path],
    #     output='screen',
    # )

    # # Gazebo client
    # gzclient = ExecuteProcess(
    #     cmd=['gzclient'],
    #     output='screen',
    #     # condition=IfCondition(LaunchConfiguration('gui')),
    # )

    # ld.add_action(gz_resource_path)
    # ld.add_action(gzserver)
    # ld.add_action(gzclient)

    # # Spawn robot
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_husky',
    #     arguments=['-entity',
    #                'husky1',
    #                '-topic',
    #                'robot_description',
    #                '-robot_namespace',
    #                'husky1',
    #                '-x', '3',
    #                '-y', '0',
    #                '-z', '0',],
    #     output='screen',
    # )

    # Launch husky_control/control.launch.py which is just robot_localization.
    # launch_husky_control = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution(
    #     [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # # the robot but does not include the joystick. Also, has a twist mux.
    # launch_husky_teleop_base = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution(
    #     [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))

    # last_action=None
    
    # for i in range(2):
    #     name_robot='husky'+str(i+1)
    #     namespace='husky'+str(i+1)
        
    #     robot_description_content = Command(
    #         [
    #             PathJoinSubstitution([FindExecutable(name="xacro")]),
    #             " ",
    #             PathJoinSubstitution(
    #                 [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
    #             ),
    #             " ",
    #             "name:="+name_robot,
    #             " ",
    #             "prefix:=''",
    #             " ",
    #             "is_sim:=true",
    #             " ",
    #             "gazebo_controllers:=",
    #             config_husky_velocity_controller,
    #         ]
    #     )
    #     robot_description = {"robot_description": robot_description_content}

    #     robot_state_publisher=Node(
    #         package="robot_state_publisher",
    #         executable="robot_state_publisher",
    #         output="screen",
    #         parameters=[{'use_sim_time': True}, robot_description,],
    #         name='robot_state_publisher',
    #         namespace=namespace,
    #         remappings=remappings,

    #     )

    #     spawn_robot = Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         name='spawn_husky'+str(i+1),
    #         arguments=['-entity',
    #                 name_robot,
    #                 '-topic',
    #                 name_robot+'/robot_description',
    #                 '-x', str(i+1),
    #                 '-y', '0',
    #                 '-z', '0',],
    #         output='screen',
    #     )

    #     if last_action is None:
    #         ld.add_action(spawn_robot)
    #         ld.add_action(robot_state_publisher)
    #         last_action=robot_state_publisher
    #     else:
    #         spawn_husky_event_handler = RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=last_action,
    #                 on_exit=[spawn_robot, robot_state_publisher],
    #             )
    #         )
    #         ld.add_action(spawn_husky_event_handler)
    #         last_action=spawn_husky_event_handler
    
    # for i in range(1):
    #     name='husky'+str(i+1)
    #     namespace='husky'+str(i+1)
    #     launch_husky_control = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution(
    #         [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])),
    #         launch_arguments={'namespace': namespace}.items()
    #     )
    #     launch_husky_teleop_base = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution(
    #         [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])),
    #         launch_arguments={'namespace': namespace}.items()
    #     )
    #     ld.add_action(launch_husky_control)
    #     ld.add_action(launch_husky_teleop_base)

    # ld.add_action(gz_resource_path)
    # ld.add_action(node_robot_state_publisher)
    # ld.add_action(spawn_joint_state_broadcaster)
    # ld.add_action(diffdrive_controller_spawn_callback)
    # ld.add_action(gzserver)
    # ld.add_action(gzclient)
    # ld.add_action(spawn_robot)
    # ld.add_action(launch_husky_control)
    # ld.add_action(launch_husky_teleop_base)


    #ultimo intento

    
    
    robots = [
        {'name': 'robot1', 'x_pos': 0.0, 'y_pos': 0.5, 'z_pos': 0.01,},
        {'name': 'robot2', 'x_pos': 0.0, 'y_pos': -0.5, 'z_pos': 0.01,},
        {'name': 'robot3', 'x_pos': 0.0, 'y_pos': -1.5, 'z_pos': 0.01,},
        {'name': 'robot4', 'x_pos': 0.0, 'y_pos': -2.5, 'z_pos': 0.01,},
    ]

    # robots = [
    #     {'name': 'robot1', 'x_pos': 0.0, 'y_pos': 0.5, 'z_pos': 0.01,},
    # ]

    arrNodes=[]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')



    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='simple',
        description='Mode to launch'
    )
    arrNodes.append(declare_mode_cmd)

<<<<<<< Updated upstream
    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
=======
    slam=LaunchConfiguration('slam')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='SLAM'
>>>>>>> Stashed changes
    )
    arrNodes.append(declare_slam_cmd)

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                            EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                default_value=''),
                                            '/usr/share/gazebo-11/models/:',
                                            str(Path(get_package_share_directory('husky_description')).
                                                parent.resolve())])
    
    arrNodes.append(gz_resource_path)

<<<<<<< Updated upstream
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
=======
>>>>>>> Stashed changes
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    arrNodes.append(gzserver)

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    arrNodes.append(gzclient)

    for robot in robots:
        namespace = robot['name'] if robot['name'] != "" else ""
        #namespace = ""
        
        config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"]
        )

        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
                ),
                " ",
                "name:="+robot['name'],
                " ",
                "prefix:="+namespace,
                " ",
                "is_sim:=true",
                # " ",
                # "gazebo_controllers:=",
                # config_husky_velocity_controller,
                " ",
                "namespace:="+namespace,
                " ",
            ]
        )
        
        params = {'robot_description': robot_description_content, 
        'use_sim_time': use_sim_time}
        
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[params],
            #remappings=[('/robot_description',namespace+'/robot_description')], #remappings,
        )

        arrNodes.append(robot_state_publisher_node)

        spawn_entity_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            #namespace=namespace,
            arguments=[
                "-topic", namespace+'/robot_description',
                '-robot_namespace', robot['name'],
                '-entity', robot['name'],
                '-x', str(robot['x_pos']),
                '-y', str(robot['y_pos']),
                '-z', str(robot['z_pos']),
                ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
        arrNodes.append(spawn_entity_cmd)

        node_tf = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[str(robot['x_pos']), str(robot['y_pos']), str(robot['z_pos']), '0', '0', '0', 
            'world', robot['name'] + 'odom'],
        output='screen')

        arrNodes.append(node_tf)

    rviz_node = Node(package    ='rviz2',
                executable ='rviz2',
                name       ='rviz2',
                output     ='log',
                arguments  =['-d', str(Path(get_package_share_directory('husky_description')) / 'rviz' / 'husky.rviz')],) 
    arrNodes.append(rviz_node)

    #gazebo_exit_event_handler = RegisterEventHandler(
    # event_handler=OnProcessExit(
    #     target_action=gazebo_node,
    #     on_exit=events.EventEmitter(event=events.Shutdown(reason='gazebo exited'))))
    # arrNodes.append(gazebo_exit_event_handler)
                        
    # rviz_exit_event_handler = RegisterEventHandler(
    # event_handler=OnProcessExit(
    #     target_action=rviz_node,
    #     on_exit=events.EventEmitter(event=events.Shutdown(reason='rviz exited'))))
    # arrNodes.append(rviz_exit_event_handler)

    #ld = LaunchDescription()

    for node in arrNodes:
        ld.add_action(node)

    return ld
