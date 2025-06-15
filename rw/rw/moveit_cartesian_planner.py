#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseArray, Pose
from xarm_msgs.srv import PlanSingleStraight, PlanExec
from rw.srv import SetProcessing  # Import our new service
import sys
import time
from collections import deque

# Service and topic names
STRAIGHT_PLAN_SERVICE = '/xarm_straight_plan'
EXEC_PLAN_SERVICE = '/xarm_exec_plan'
PERCEPTION_SERVICE = 'set_perception_processing'
POSE_ARRAY_TOPIC = '/arm_path_poses'

class CartesianPlannerClient(Node):
    def __init__(self):
        super().__init__('cartesian_planner_client')
        self.get_logger().info("Initializing Cartesian Planner Client...")
        
        self.callback_group = ReentrantCallbackGroup()
        self.is_initialized = False
        self._is_executing = False
        self.path_queue = deque()

        self.perception_client = self.create_client(SetProcessing, PERCEPTION_SERVICE, callback_group=self.callback_group)
        self.straight_plan_client = self.create_client(PlanSingleStraight, STRAIGHT_PLAN_SERVICE, callback_group=self.callback_group)
        self.exec_plan_client = self.create_client(PlanExec, EXEC_PLAN_SERVICE, callback_group=self.callback_group)

        for client, name in [(self.straight_plan_client, STRAIGHT_PLAN_SERVICE),
                             (self.exec_plan_client, EXEC_PLAN_SERVICE),
                             (self.perception_client, PERCEPTION_SERVICE)]:
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'Service "{name}" not available. Shutting down.')
                return

        self.get_logger().info("All required services are available.")

        self.pose_array_sub = self.create_subscription(
            PoseArray, POSE_ARRAY_TOPIC, self.pose_array_callback, 10, callback_group=self.callback_group
        )
        
        self.is_initialized = True
        self.get_logger().info(f"Ready and waiting for paths on '{POSE_ARRAY_TOPIC}'.")

    def set_perception(self, state: bool):
        req = SetProcessing.Request(data=state)
        self.get_logger().info(f"Requesting to {'ENABLE' if state else 'PAUSE'} perception...")
        future = self.perception_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result():
            self.get_logger().info(f"Perception gate response: {future.result().message}")
        else:
            self.get_logger().error("Failed to call perception gate service.")

    def pose_array_callback(self, msg: PoseArray):
        if self._is_executing:
            self.get_logger().warn("Currently executing a path. Skipping new PoseArray.")
            return
        if not msg.poses:
            self.get_logger().warn("Received an empty PoseArray message.")
            return
            
        self._is_executing = True
        self.set_perception(False)
        self.get_logger().info(f"Received path with {len(msg.poses)} poses. Starting sequence.")
        
        start_pose = Pose()
        start_pose.position.x, start_pose.position.y, start_pose.position.z = 0.28, 0.0, 0.25
        start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w = 1.0, 0.0, 0.0, 0.0
        
        home_pose = Pose()
        home_pose.position.x, home_pose.position.y, home_pose.position.z = 0.2, 0.0, 0.4
        home_pose.orientation.x, home_pose.orientation.y, home_pose.orientation.z, home_pose.orientation.w = 1.0, 0.0, 0.0, 0.0

        self.path_queue = deque([start_pose] + msg.poses + [home_pose])
        self.process_next_pose()

    def process_next_pose(self):
        if not self.path_queue:
            self.get_logger().info("--- Path execution complete. ---")
            self.finish_sequence()
            return

        target_pose = self.path_queue.popleft()
        self.get_logger().info(f"Planning for next waypoint ({len(self.path_queue)} remaining)...")
        plan_req = PlanSingleStraight.Request(target=target_pose)
        plan_future = self.straight_plan_client.call_async(plan_req)
        plan_future.add_done_callback(self.plan_done_callback)

    def plan_done_callback(self, plan_future):
        try:
            plan_result = plan_future.result()
            if not plan_result.success:
                self.get_logger().error("Cartesian planning failed! Aborting sequence.")
                self.finish_sequence()
                return
        except Exception as e:
            self.get_logger().error(f"Exception during planning: {e}. Aborting sequence.")
            self.finish_sequence()
            return

        self.get_logger().info("Planning successful. Executing trajectory...")
        exec_req = PlanExec.Request(wait=True)
        exec_future = self.exec_plan_client.call_async(exec_req)
        exec_future.add_done_callback(self.exec_done_callback)

    def exec_done_callback(self, exec_future):
        try:
            exec_result = exec_future.result()
            if not exec_result.success:
                self.get_logger().error("Trajectory execution failed! Aborting sequence.")
                self.finish_sequence()
                return
        except Exception as e:
            self.get_logger().error(f"Exception during execution: {e}. Aborting sequence.")
            self.finish_sequence()
            return

        self.get_logger().info("Waypoint execution successful.")
        time.sleep(0.5) 
        self.process_next_pose()
        
    def finish_sequence(self):
        self.set_perception(True)
        self.path_queue.clear()
        self._is_executing = False

def main(args=None):
    rclpy.init(args=args)
    planner_client_node = CartesianPlannerClient()
    if not planner_client_node.is_initialized:
        print("[ERROR] CartesianPlannerClient failed to initialize. Shutting down.", file=sys.stderr)
    else:
        try:
            rclpy.spin(planner_client_node)
        except KeyboardInterrupt:
            planner_client_node.get_logger().info("Keyboard interrupt, shutting down.")
    
    if rclpy.ok():
        planner_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()