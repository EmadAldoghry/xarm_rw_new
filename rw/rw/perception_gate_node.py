#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from rw.srv import SetProcessing  # Import our new service

class PerceptionGateNode(Node):
    def __init__(self):
        super().__init__('perception_gate_node')
        
        self.is_processing_enabled = True  # Start in the active state

        # Service to enable/disable processing
        self.srv = self.create_service(
            SetProcessing,
            'set_perception_processing',
            self.set_processing_callback
        )

        # Subscriber to the raw point cloud from the camera
        self.raw_cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera2/points',  # The original source topic
            self.cloud_callback,
            qos_profile_sensor_data
        )

        # Publisher for the "gated" point cloud
        self.gated_cloud_pub = self.create_publisher(
            PointCloud2,
            'gated_point_cloud',  # The new topic for crack_path_node to listen to
            10
        )

        self.get_logger().info('Perception Gate Node has started. Processing is ENABLED.')

    def set_processing_callback(self, request, response):
        self.is_processing_enabled = request.data
        if self.is_processing_enabled:
            response.message = "Perception processing has been ENABLED."
            self.get_logger().info(response.message)
        else:
            response.message = "Perception processing has been PAUSED."
            self.get_logger().info(response.message)
        
        response.success = True
        return response

    def cloud_callback(self, msg: PointCloud2):
        # Only forward the message if processing is enabled
        if self.is_processing_enabled:
            self.gated_cloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionGateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()