from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("MyCo-3.5-0.95", package_name="myco_3_5_950mm_ros2_moveit2").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
