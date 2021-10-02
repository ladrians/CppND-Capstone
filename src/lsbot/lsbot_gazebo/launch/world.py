import sys
import os
import launch

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess

def read_file(path):
    with open(path, 'r') as f:
        contents = f.read()
    return contents

def generate_launch_description():
    output_mode = 'both'
    project_name = 'lsbot_gazebo'
    description_name = 'lsbot_description'
    gpu = False
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_bot = get_package_share_directory(project_name)
    dsc_bot = get_package_share_directory(description_name)
    rviz_config_dir = os.path.join(
        dsc_bot,
        'rviz',
        'visualization.rviz')

    urdf_dir =  os.path.join(dsc_bot,
        'urdf')
    xacro_urdf = os.path.join(urdf_dir, 'sandbox_bot.xacro')
    robot_urdf = os.path.join(urdf_dir, 'sandbox_bot.urdf')
    xacro_proc = subprocess.Popen("xacro --inorder {0} gpu:={1} > {2}".format(xacro_urdf, gpu, robot_urdf),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    xacro_proc.wait()
    assert os.path.exists(robot_urdf)
    urdf_contents = read_file(robot_urdf)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    install_dir = get_package_prefix(project_name)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if (key.isupper()):
                envs[key] = os.environ[key]
    except Exception as e:
        print("Error with Envs: " + str(e))
        return None

    spawn_entity_message_contents = "'{initial_pose:{ position: {x: 0, y: 0, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}},  name: \"robot_description\", xml: \"" + urdf_contents.replace('"', '\\"') + "\"}'"
    spawn_entity = launch.actions.ExecuteProcess(
        name='spawn_entity', cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_entity_message_contents], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)

    # Publishes the URDF to /robot_description.
    # This is a workaround to make rviz get the urdf.
    urdf_pub_data = urdf_contents.replace('"', '\\"')
    launch_urdf = launch.actions.ExecuteProcess(
        name='launch_urdf', cmd=['ros2', 'topic', 'pub', '-r', '0.1', '/robot_description', 'std_msgs/String', '\'data: "' + urdf_pub_data + '"\''], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)

    ld = LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_bot, 'worlds', 'corridor_with_obstacles.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('static_tf', default_value='true',
                              description='publish static_tf for wheels'),
        gazebo,
        spawn_entity,
        launch_urdf,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output=output_mode,
            arguments=[robot_urdf]),
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output=output_mode),
        Node(
            name='left_bogie_front_wheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.10805', '0.110583', '-0.09375', '0', '0', '1.5708', 'body', 'left_bogie_front_wheel'],
            condition=IfCondition(LaunchConfiguration('static_tf')),
            output=output_mode),
        Node(
            name='left_bogie_rear_wheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0143', '0.110583', '-0.09375', '0', '0', '1.5708', 'body', 'left_bogie_rear_wheel'],
            condition=IfCondition(LaunchConfiguration('static_tf')),
            output=output_mode),
        Node(
            name='left_rocker_rear_wheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.10805', '0.110583', '-0.09375', '0', '0', '1.5708', 'body', 'left_rocker_rear_wheel'],
            condition=IfCondition(LaunchConfiguration('static_tf')),
            output=output_mode),

        Node(
            name='right_bogie_front_wheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.10805', '-0.110583', '-0.09375', '0', '0', '1.5708', 'body', 'right_bogie_front_wheel'],
            condition=IfCondition(LaunchConfiguration('static_tf')),
            output=output_mode),
        Node(
            name='right_bogie_rear_wheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0143', '-0.110583', '-0.09375', '0', '0', '1.5708', 'body', 'right_bogie_rear_wheel'],
            condition=IfCondition(LaunchConfiguration('static_tf')),
            output=output_mode),
        Node(
            name='right_rocker_rear_wheel',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.10805', '-0.110583', '-0.09375', '0', '0', '1.5708', 'body', 'right_rocker_rear_wheel'],
            condition=IfCondition(LaunchConfiguration('static_tf')),
            output=output_mode),
    ])
    return ld
'''
  <node name="controller_spawner" pkg="controller_manager" type="spawner"
        args="rover_joint_publishre rover_velocity_controller --shutdown-timeout 2" />
  <node pkg="topic_tools" type="relay" name="cmd_vel_relay" args="cmd_vel rover_velocity_controller/cmd_vel" />
'''
