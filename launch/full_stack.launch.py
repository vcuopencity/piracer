# Standard library imports
from os.path import join

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """ Launch hardware_nodes.launch and autonomy_manager node, as well as every control mode of the piracer using each
    respective launch file, passing launch_hardware=false so there aren't duplicates of the hardware nodes running.
    """
    car_config = join(get_package_share_directory('piracer'),
                      'config', 'car_config.yaml')
    ekf_config = join(get_package_share_directory('piracer'),
                      'config', 'ekf_config.yaml')
    piracer_launch_directory = get_package_share_directory('piracer')
    bridge_launch_directory = get_package_share_directory('mqtt_bridge')
    launch_bridge = LaunchConfiguration('launch_bridge')

    urdf_file_name = 'piracer_urdf.xml'
    urdf = join(get_package_share_directory('piracer'),
                      'config', urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name',
            default_value='car1',
            description='Sets the namespace for this car.'),
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Determinds if appropriate bridge files are launched from the mqtt_bridge package.'
        ),
        IncludeLaunchDescription(

            PythonLaunchDescriptionSource([piracer_launch_directory, '/ackermann_control.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name'),
                'launch_hardware': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/teleop_control.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name'),
                'launch_hardware': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/open_loop_control.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name'),
                'launch_ackermann': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/command_bridge.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
            condition=IfCondition(launch_bridge)
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='autonomy_manager',
            name='autonomy_manager',
            parameters=[car_config]
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='v2x_node',
            name='v2x_node',
            parameters=[car_config]
        ),
        Node(
            package='robot_localization',
            namespace=[LaunchConfiguration('agent_name')],
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        Node(
            package='robot_state_publisher',
            namespace=[LaunchConfiguration('agent_name')],
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]
        )
    ])
