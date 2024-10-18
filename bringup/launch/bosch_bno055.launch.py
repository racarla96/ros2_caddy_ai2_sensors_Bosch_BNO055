from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# https://github.com/ros-controls/ros2_control_demos/blob/master/example_5/bringup/launch/rrbot_system_with_external_sensor.launch.py

def generate_launch_description():

    bosch_bno055_sensors = PathJoinSubstitution(
        [
            FindPackageShare("ros2_caddy_ai2_sensors_bosch_bno055"),
            "config",
            "bosch_bno055_sensors.yaml",
        ]
    )
    

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[bosch_bno055_sensors],
        output="both",
    )

    bosch_bno055_sensors_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bosch_bno055_sensor_configuration", "--param-file", bosch_bno055_sensors],
    )

    nodes = [
        ros2_control_node,
        bosch_bno055_sensors_spawner
    ]

    return LaunchDescription(nodes)