import os
import yaml
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

robot_names = []
robot_positions = []
sim_pos=[]

with open(os.path.join(get_package_share_directory('husky_navigation'), 'params', 'multirobot_names.yaml'), 'r') as file:
    names = yaml.safe_load(file)
    for i in range(len(names['names'])):
        robot_names.append(names['names'].get('robot'+str(i+1)))
        robot_positions.append(names['position'].get('robot'+str(i+1)))

with open(os.path.join(get_package_share_directory('cv_gdal'), 'maps', 'map1.yaml'), 'r') as file:
    map_conf = yaml.safe_load(file)
    sim_pos.append(map_conf['origin'])
    #sim_pos contains the position of the first robot in the map (x,y,z)

with open(os.path.join(get_package_share_directory('cv_gdal'), 'worlds', 'la_cabrera_5D.world'), 'r') as file:
    world_content = file.read()
    start_index = world_content.find('<size>')
    end_index = world_content.find('</size>')
    size_values = world_content[start_index + len('<size>'):end_index].strip().split()
    #size_values contains the size of the world (x,y,z)


def generate_launch_description():

    ld = LaunchDescription(ARGUMENTS)

    robots = [{} for i in range(len(robot_names))]
    for i in range(len(robot_names)):
        robots[i]['name'] = robot_names[i]
        robots[i]['x_pos'] = robot_positions[i].get('x')
        robots[i]['y_pos'] = robot_positions[i].get('y')
        robots[i]['z_pos'] = robot_positions[i].get('z')


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
        arguments=[str(robot['x_pos']),str(robot['y_pos']),'0', '0', '0', '0', 
            '/map', robot['name'] + '/map'],    
        output='screen')

        arrNodes.append(node_tf)

        odom_tf = Node( 
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[str(-robot['x_pos']),str(-robot['y_pos']), '0', '0', '0', '0', 
            robot['name']+'/map', robot['name'] + '/odom'],    
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
