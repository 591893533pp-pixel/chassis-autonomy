"""Localization + navigation only. Never starts CAN or a velocity-to-hardware bridge.
Requires the separate dual sensor + odometry process; cannot run beside SAM mapping.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
ROOT=Path(__file__).resolve().parents[1]
def setup(context):
    path=Path(LaunchConfiguration('map').perform(context)).resolve()
    if not path.is_file():raise RuntimeError('Provide an exported, inspected map YAML')
    cfg=str(ROOT/'config/nav2.yaml');nodes=[]
    specs=[('map_server','nav2_map_server'),('amcl','nav2_amcl'),('planner_server','nav2_planner'),('controller_server','nav2_controller'),('bt_navigator','nav2_bt_navigator'),('collision_monitor','nav2_collision_monitor')]
    for name,pkg in specs:
        override={'use_sim_time':False}
        if name=='map_server':override['yaml_filename']=str(path)
        if name=='bt_navigator':override['default_nav_to_pose_bt_xml']=str(ROOT/'config/navigate_to_pose.xml')
        nodes.append(Node(package=pkg,executable=name,name=name,output='screen',parameters=[cfg,override],remappings=[('cmd_vel','/chassis_nav/cmd_vel_raw')]))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='chassis_nav_lifecycle',parameters=[{'autostart':True,'node_names':[n for n,_ in specs]}]))
    nodes.append(ExecuteProcess(cmd=['python3',str(ROOT/'scripts/nav_sensor_bridge.py')],output='screen'))
    return nodes
def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('map'),OpaqueFunction(function=setup)])
