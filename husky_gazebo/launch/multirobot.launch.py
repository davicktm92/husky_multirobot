from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    ld = LaunchDescription(ARGUMENTS)
    
    robots = [
        {'name': 'robot1', 'x_pos': 0.0, 'y_pos': 0.5, 'z_pos': 0.01,},
        {'name': 'robot2', 'x_pos': 0.0, 'y_pos': -0.5, 'z_pos': 0.01,},
        {'name': 'robot3', 'x_pos': 0.0, 'y_pos': -1.5, 'z_pos': 0.01,},
        {'name': 'robot4', 'x_pos': 0.0, 'y_pos': -2.5, 'z_pos': 0.01,},
    ]

    arrNodes=[]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_path = LaunchConfiguration('world_path')

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                            EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                default_value=''),
                                            '/usr/share/gazebo-11/models/:',
                                            str(Path(get_package_share_directory('husky_description')).
                                                parent.resolve())])
    
    arrNodes.append(gz_resource_path)

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
    )

    arrNodes.append(gzclient)

    for robot in robots:
        namespace = robot['name'] if robot['name'] != "" else ""
        prefix = namespace + "/" if namespace != "" else ""

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
                "prefix:="+prefix,
                " ",
                "is_sim:=true",
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
        )

        arrNodes.append(robot_state_publisher_node)

        spawn_entity_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
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
        arguments=[str(robot['x_pos']),str(robot['y_pos']),str(robot['z_pos']), '0', '0', '0', 
            '/map', robot['name'] + '/map'],    
        output='screen')

        arrNodes.append(node_tf)

        odom_tf = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 
            '/map', robot['name'] + '/odom'],    
        output='screen')

        arrNodes.append(odom_tf)

    rviz_node = Node(package    ='rviz2',
                executable ='rviz2',
                name       ='rviz2',
                output     ='log',
                arguments  =['-d', str(Path(get_package_share_directory('husky_description')) / 'rviz' / 'husky_multi.rviz')],) 
    arrNodes.append(rviz_node)

    for node in arrNodes:
        ld.add_action(node)

    return ld
