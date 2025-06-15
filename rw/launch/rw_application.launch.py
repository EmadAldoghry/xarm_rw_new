from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the nodes for the crack detection and following pipeline,
    including a perception gate and a dedicated visualizer.
    """
    
    # 1. Gatekeeper for perception data
    perception_gate_node = Node(
        package='rw',
        executable='perception_gate_node',
        name='perception_gate_node',
        output='screen'
    )

    # 2. Point cloud processing node (now subscribes to the gated topic)
    crack_path_node = Node(
        package='rw',
        executable='crack_path_node',
        name='crack_path_node',
        output='screen'
    )

    # 3. Node for generating poses from the processed point cloud
    fake_poses_node = Node(
        package='rw',
        executable='fake_poses',
        name='fake_poses',
        output='screen'
    )

    # 4. Dedicated node for visualizing the generated path in RViz
    pose_visualizer_node = Node(
        package='rw',
        executable='pose_visualizer_node',
        name='pose_visualizer_node',
        output='screen'
    )

    # 5. The motion planner that controls the arm and the perception gate
    moveit_cartesian_planner_node = Node(
        package='rw',
        executable='moveit_cartesian_planner',
        name='moveit_cartesian_planner',
        output='screen'
    )

    return LaunchDescription([
        perception_gate_node,
        crack_path_node,
        fake_poses_node,
        pose_visualizer_node,
        moveit_cartesian_planner_node,
    ])