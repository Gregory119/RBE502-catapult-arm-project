from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5", package_name="catapult_moveit_config_generated").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
