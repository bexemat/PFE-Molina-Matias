import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Variable de red global
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='0'),

        # 1. Agente micro-ROS (Cargando su workspace específico)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'source ~/microros_ws/install/setup.bash && '
                'export ROS_LOCALHOST_ONLY=0 && '
                'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200'
            ],
            output='screen'
        ),

        # 2. Orquestador Central
        Node(
            package='robot_control',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
            respawn=True
        ),

        # 3. Planificador
        Node(
            package='robot_control',
            executable='planner_node',
            name='trajectory_planner_node',
            output='screen'
        ),

        # 4. Interfaz Gráfica (UI)
        Node(
            package='robot_control',
            executable='robot_ui_node',
            name='robot_ui_node',
            output='screen'
        ),
    ])