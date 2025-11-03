import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):

    # Coordinator node
    coordinator_node = Node(
        package='multi_vehicle_coordinator',
        executable='coordinator_node',
        name='multi_vehicle_coordinator',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Vehicle 3: Lift&Cruise with square flight (151m altitude)
    square_flight_node = Node(
        package='custom_square_flight',
        executable='custom_square_flight',
        name='vehicle3_square_flight',
        output='screen',
        parameters=[
            {'px4_sysid': 4},
            {'reverse_direction': True},
            {'square_size': 300.0},
            {'takeoff_altitude': 151.0},
            {'use_sim_time': True}
        ]
    )

    # Vehicle 4: Drone with line flight (33m altitude)
    line_flight_node_4 = Node(
        package='custom_line_flight',
        executable='custom_line_flight',
        name='vehicle4_line_flight',
        output='screen',
        parameters=[
            {'px4_sysid': 3},
            {'line_distance': 100.0},
            {'takeoff_altitude': 33.0},
            {'use_sim_time': True}
        ]
    )

    # Vehicle 5: Drone with line flight (33m altitude)
    line_flight_node_5 = Node(
        package='custom_line_flight',
        executable='custom_line_flight',
        name='vehicle5_line_flight',
        output='screen',
        parameters=[
            {'px4_sysid': 11},
            {'line_distance': 100.0},
            {'takeoff_altitude': 33.0},
            {'use_sim_time': True}
        ]
    )

    # Vehicle 6: Drone with line flight (33m altitude)
    line_flight_node_6 = Node(
        package='custom_line_flight',
        executable='custom_line_flight',
        name='vehicle6_line_flight',
        output='screen',
        parameters=[
            {'px4_sysid': 12},
            {'line_distance': 100.0},
            {'takeoff_altitude': 33.0},
            {'use_sim_time': True}
        ]
    )

    # Vehicle 7: Drone with line flight (33m altitude)
    line_flight_node_7 = Node(
        package='custom_line_flight',
        executable='custom_line_flight',
        name='vehicle7_line_flight',
        output='screen',
        parameters=[
            {'px4_sysid': 13},
            {'line_distance': 100.0},
            {'takeoff_altitude': 33.0},
            {'use_sim_time': True}
        ]
    )

    nodes_to_start = [
        # coordinator_node,
        square_flight_node,
        line_flight_node_4,
        line_flight_node_5,
        line_flight_node_6,
        line_flight_node_7
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
