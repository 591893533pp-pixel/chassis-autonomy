from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[1]
def setup(context):
    full=LaunchConfiguration('full_mesh').perform(context).lower()=='true'
    path=ROOT/'model'/('chassis_visual.urdf' if full else 'chassis_proxy.urdf')
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',name='chassis_description',parameters=[{'robot_description':path.read_text()}]),
            Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0','--frame-id','base_link_trial','--child-frame-id','chassis_model_base']),
            Node(package='rviz2',executable='rviz2',arguments=['-d',str(ROOT/'rviz/chassis.rviz'),'-f',LaunchConfiguration('fixed_frame').perform(context)]),
            ExecuteProcess(cmd=['python3',str(ROOT/'scripts/display_footprint.py')])]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('full_mesh',default_value='false'),DeclareLaunchArgument('fixed_frame',default_value='trial_map'),OpaqueFunction(function=setup)])
