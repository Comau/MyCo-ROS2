from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("MyCo-10-1.00", package_name="myco_10_1000mm_ros2_moveit2").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
