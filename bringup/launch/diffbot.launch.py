# Copyright 2020 ros2_control Development Team
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
import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "diffdrive_arduino"
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffdrive_arduino"), "urdf", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_arduino"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )
    lidar_spawner = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Express'
            }
        ]
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        lidar_spawner,
        twist_mux
    ]

    return LaunchDescription(nodes)
