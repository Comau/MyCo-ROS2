from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("MyCo-8-1.30", package_name="myco_8_1300mm_ros2_moveit2").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
