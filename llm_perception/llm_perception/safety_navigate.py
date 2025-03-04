#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped

class BehaviorTreeLogListener(Node):
    def __init__(self):
        super().__init__('behavior_tree_log_listener')
        # Subscriber for behavior tree log messages
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.listener_callback,
            1
        )
        self.get_logger().info("Subscribed to /behavior_tree_log.")

        # Publisher for goal_pose messages
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info("Goal publisher created on topic 'goal_pose'.")

        # Define the hard-coded safety point
        self.safety_point = PoseStamped()
        self.safety_point.header.frame_id = 'map'
        # Set the time later when publishing
        self.safety_point.pose.position.x = -0.03473655879497528 
        self.safety_point.pose.position.y = -1.853783130645752
        self.safety_point.pose.position.z = 0.0
        # Identity quaternion (no rotation)
        self.safety_point.pose.orientation.x = 0.0
        self.safety_point.pose.orientation.y = 0.0
        self.safety_point.pose.orientation.z = 0.0
        self.safety_point.pose.orientation.w = 1.0

    def listener_callback(self, msg):
        # Print the full message so you can check its format
        event = msg.event_log[0]
        # Check the condition: if node is 'ComputePathToPose' and current_status is 'FAILURE'
        if (event.node_name == 'FollowPath' and event.current_status == 'FAILURE') or (event.node_name == 'ComputePathToPose' and event.current_status == 'FAILURE'): 
            self.get_logger().info("Goal is failed. Publishing safety point.")
            print()

            # Update the header stamp to current time
            self.safety_point.header.stamp = self.get_clock().now().to_msg()
            # Publish the safety point to 'goal_pose'
            self.goal_pub.publish(self.safety_point)
        else:
            # For debugging, you may log other messages if desired.
            self.get_logger().debug(
                f"Node: {event.node_name}, Previous: {event.previous_status}, Current: {event.current_status}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeLogListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
