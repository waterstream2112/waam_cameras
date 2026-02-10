#! /usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.conditions import IfCondition


configurable_parameters = [{'name': 'namespace',    'default': 'waam', 'description': 'Set namespace for the odom using env variable'},
                           {'name': 'config_file',  'default': 'config/waam_devices_params.yaml', 'description': 'Params config file'}]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
    
    
def launch_setup(context):
    
    #--- Find the package share directory
    pkg_share_dir = get_package_share_directory('waam_cameras')
    
    #--- Get params value from launch arguments
    launch_args = dict([(param['name'], LaunchConfiguration(param['name'])) for param in configurable_parameters])
    
    #--- Read params value from config file
    config_file   = PathJoinSubstitution([pkg_share_dir, LaunchConfiguration('config_file')])
    rviz_cfg      = PathJoinSubstitution([pkg_share_dir, LaunchConfiguration('rviz_cfg')])
    cam_intrinsic = PathJoinSubstitution([pkg_share_dir, LaunchConfiguration('cam_intrinsic')])
    cam_extrinsic = PathJoinSubstitution([pkg_share_dir, LaunchConfiguration('cam_extrinsic')])
    
    #--- Set namespace
    namespace=EnvironmentVariable(LaunchConfiguration('namespace')).perform(context)
    
    #--- Read params directly from YAML config file
    params_from_file =  yaml_to_dict(config_file.perform(context))
    # topic_out_raw_image     = params_from_file['/**']['ros__parameters']['topic_out_raw_image']
    # topic_out_thermal_image = params_from_file['/**']['ros__parameters']['topic_out_thermal_image']

    #--- Remapping
    remappings = [('/tf',        'tf'),
                  ('/tf_static', 'tf_static'),
                #   ('topic_out_raw_image',       topic_out_raw_image), 
                #   ('topic_out_thermal_image',   topic_out_thermal_image),
                  ]
    

    #--- Launch Baumer IR Camera node
    node_baumer_ir_cam = Node(
        package='waam_cameras',
        executable='node_baumer_ir_cam',
        name='baumer_ir_cam_node',
        namespace=namespace,
        remappings=remappings,
        parameters=[launch_args,
                    config_file,
                    {'cam_intrinsic': cam_intrinsic},
                    {'cam_extrinsic': cam_extrinsic}],
        output='screen'
    )
    
    #--- Launch rviz
    remappings = [('/tf',        namespace + '/tf'),
                  ('/tf_static', namespace + '/tf_static')]
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        remappings=remappings,
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return [node_baumer_ir_cam]


def generate_launch_description():
    
    #--- Declare launch arguments
    launch_arguments = [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in configurable_parameters] 
    
    #--- Declare launch_setup
    launch_func = [OpaqueFunction(function=launch_setup)]

    #--- Create and return LaunchDescription
    ld = LaunchDescription(launch_arguments + launch_func)

    return ld


