from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the three custom nodes for the crack detection pipeline.
    """
    
    # Node for the crack path processing
    crack_path_node = Node(
        package='rw',
        executable='crack_path_node',
        name='crack_path_node',
        output='screen',
    )

    # Node for publishing fake poses for testing
    fake_poses_node = Node(
        package='rw',
        executable='fake_poses',
        name='fake_poses',
        output='screen',
    )

    # Node for the cartesian planner
    moveit_cartesian_planner_node = Node(
        package='rw',
        executable='moveit_cartesian_planner',
        name='moveit_cartesian_planner',
        output='screen',
    )

    return LaunchDescription([
        crack_path_node,
        fake_poses_node,
        moveit_cartesian_planner_node,
    ])