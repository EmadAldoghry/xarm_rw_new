#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

class PoseVisualizerNode(Node):
    def __init__(self):
        super().__init__('pose_visualizer_node')
        
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/arm_path_poses',
            self.pose_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.get_logger().info('Pose Visualizer Node started.')

    def pose_callback(self, msg: PoseArray):
        marker_array = MarkerArray()
        
        # Create a line strip marker to connect the poses
        line_strip = Marker()
        line_strip.header = msg.header
        line_strip.ns = "path_line"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.01  # Line width
        line_strip.color.a = 1.0
        line_strip.color.r = 0.0
        line_strip.color.g = 1.0
        line_strip.color.b = 0.0
        line_strip.lifetime = Duration(seconds=0).to_msg() # Persist forever

        for i, pose in enumerate(msg.poses):
            # Add points to the line strip
            line_strip.points.append(pose.position)
            
            # Create an arrow marker for each pose
            arrow = Marker()
            arrow.header = msg.header
            arrow.ns = "path_arrows"
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose
            arrow.scale.x = 0.05  # Arrow length
            arrow.scale.y = 0.01  # Arrow width
            arrow.scale.z = 0.01  # Arrow height
            arrow.color.a = 1.0
            arrow.color.r = 1.0
            arrow.color.g = 0.0
            arrow.color.b = 0.0
            arrow.lifetime = Duration(seconds=0).to_msg() # Persist forever
            marker_array.markers.append(arrow)

        marker_array.markers.append(line_strip)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} markers for visualization.')

def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()