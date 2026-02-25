
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myco_basic_api',
            executable='myco_gui.py',
            name='myco_gui_node',
            output='screen',
            parameters=[
                {'use_fake_robot': False},
                {'use_sim_time': False}
            ]
        )
    ])