#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile  # Import QoSProfile to adjust buffering
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped

class BehaviorTreeLogListener(Node):
    def __init__(self):
        super().__init__('behavior_tree_log_listener')
        # Use a custom QoS with increased depth to buffer more messages
        qos_profile = QoSProfile(depth=100)
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info("Subscribed to /behavior_tree_log with QoS depth 100.")

        # Publisher for goal_pose messages
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info("Goal publisher created on topic 'goal_pose'.")

        # Define the hard-coded safety point
        self.safety_point = PoseStamped()
        self.safety_point.header.frame_id = 'map'
        # Position for the safety point
        self.safety_point.pose.position.x = -1.95
        self.safety_point.pose.position.y = -0.05
        self.safety_point.pose.position.z = 0.0
        # Orientation: identity quaternion (no rotation)
        self.safety_point.pose.orientation.x = 0.0
        self.safety_point.pose.orientation.y = 0.0
        self.safety_point.pose.orientation.z = 0.0
        self.safety_point.pose.orientation.w = 1.0

    def listener_callback(self, msg):
        # Loop over all events in the BehaviorTreeLog message
        for event in msg.event_log:
            # Check if the event is from the 'GoalReached' node and its status is FAILURE
            if event.node_name == "NavigateWithReplanning" and event.current_status == "FAILURE":
                self.get_logger().info("GoalReached returned FAILURE. Publishing safety point.")
                # Update the stamp before publishing
                self.safety_point.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub.publish(self.safety_point)
            elif event.node_name == "NavigateWithReplanning" and event.current_status == "SUCCESS":
                self.get_logger().info("GoalReached returned SUCCESS.")
            else:
                # Log other events at the debug level for further analysis
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
