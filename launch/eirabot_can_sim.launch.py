from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

import os

def generate_launch_description():
    can_interface_name = "vcan0"
    package_name = "dunker_motor_control"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "urdf/eirabot_controller",
                    "robot_controller.urdf.xacro",
                ]
            ),
            " ",
            "can_interface_name:=",
            can_interface_name,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config/eirabot_control", "ros2_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    turntable_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    turntable_control_node = Node(
        package="dunker_motor_control",
        executable="turntable_control_node",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    twist_mux_params = PathJoinSubstitution([FindPackageShare(package_name), 'config/eirabot_control', 'twist_mux.yaml'])

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )

    slave_config_n5 = PathJoinSubstitution(
        [FindPackageShare(package_name), "config/eirabot_control", "cia402_slave_n5.eds"]
    )

    slave_config_n6 = PathJoinSubstitution(
        [FindPackageShare(package_name), "config/eirabot_control", "cia402_slave_n6.eds"]
    )

    slave_config_n4 = PathJoinSubstitution(
        [FindPackageShare(package_name), "config/eirabot_control", "cia402_slave_n4.eds"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "5",
            "node_name": "left_wheel_slave",
            "slave_config": slave_config_n5,
        }.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "6",
            "node_name": "right_wheel_slave",
            "slave_config": slave_config_n6,
        }.items(),
    )
    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "4",
            "node_name": "turntable_slave",
            "slave_config": slave_config_n4,
        }.items(),
    )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("eirabot_can_bus"), "launch", "basic.rviz"]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[
            {'robot_description': robot_description},
        ]
    )
    
    camera_transformer_node = Node(
        package='canopen_pgv150i_tests',
        executable='camera_node'
    )

    qr_mapper_node = Node (
        package='demo_area_mapping',
        executable= 'qr_mapper'
    )
    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        slave_node_1,
        slave_node_2,
        slave_node_3,
        robot_controller_spawner,
        robot_state_publisher_node,
        twist_mux,
        rviz2,
        turntable_controller_spawner,
        turntable_control_node,
        # camera_transformer_node,
        qr_mapper_node
    ]
    
    os.system('sudo modprobe vcan')
    os.system('sudo ip link add dev vcan0 type vcan')
    os.system('sudo ip link set vcan0 txqueuelen 1000')
    os.system('sudo ip link set up vcan0')
    
    return LaunchDescription(nodes_to_start)
