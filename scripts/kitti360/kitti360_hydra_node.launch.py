#!/usr/bin/env python3
"""Standalone ROS2 launch file for KITTI-360 hydra + visualizer."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    adt4_ws = os.environ['ADT4_WS']
    output_dir = os.environ['ADT4_OUTPUT_DIR']
    scripts = f'{adt4_ws}/src/awesome_dcist_t4/scripts/kitti360'

    hydra_node = Node(
        package='hydra_ros',
        executable='hydra_ros_node',
        name='hydra',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('~/input/camera/rgb/image_raw', '/kitti360/rgb/image_raw'),
            ('~/input/camera/depth_registered/image_rect', '/kitti360/depth/depth_registered'),
            ('~/input/camera/rgb/camera_info', '/kitti360/rgb/camera_info'),
        ],
        arguments=[
            '--config-utilities-file', f'{scripts}/kitti360_ros_input.yaml',
            '--config-utilities-file', f'{scripts}/kitti360_hydra.yaml',
            '--config-utilities-yaml', '{robot_id: 0}',
            '--config-utilities-yaml', '{odom_frame: world}',
            '--config-utilities-yaml', '{robot_frame: cam0}',
            '--config-utilities-yaml', '{map_frame: map}',
            '--config-utilities-yaml', f'{{log_path: {output_dir}/hydra}}',
            '--config-utilities-yaml', '{output: {use_timestamp: false,overwrite: true}}',
            '--config-utilities-yaml', '{glog_level: 0}',
            '--config-utilities-yaml', '{glog_verbosity: 0}',
        ],
        output='screen',
    )

    visualizer_node = Node(
        package='hydra_visualizer',
        executable='hydra_visualizer_node',
        name='hydra_visualizer',
        parameters=[{'use_sim_time': True}],
        remappings=[('~/dsg', '/hydra/backend/dsg')],
        arguments=[
            '--config-utilities-yaml', '{glog_level: 0}',
            '--config-utilities-yaml', '{glog_verbosity: 1}',
        ],
        output='screen',
    )

    return LaunchDescription([hydra_node, visualizer_node])
