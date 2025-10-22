import os
import random
import yaml
import ast

from jinja2 import Environment, FileSystemLoader

from collections import defaultdict

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart, OnProcessExit


def launch_setup(context, *args, **kwargs):
    
    drone_node = Node(
        package='custom_flight',
        executable='custom_square_flight',
        name='drone_control',
        parameters=[
            {'px4_sysid': 3},
            {'reverse_direction': False},
            {'square_size': 100.0},
            {'use_sim_time': True}
        ]
    )

    lc_62_node = Node(
        package='custom_flight',
        executable='custom_square_flight',
        name='lc_62_control',
        parameters=[
            {'px4_sysid': 4},
            {'reverse_direction': True},
            {'square_size': 300.0},
            {'use_sim_time': True}
        ]
    )

    nodes_to_start = [
        drone_node,
        lc_62_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
