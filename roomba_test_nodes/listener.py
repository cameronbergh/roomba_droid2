import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class ListenerNode(Node):
    def __init__(self):
        super().__init__('roomba_listener_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.received_messages = 0
        self.max_messages = 5 # Listen for 5 messages
        self.shutdown_timer_duration = 10 # seconds, as a fallback
        self.shutdown_timer = self.create_timer(self.shutdown_timer_duration, self.timed_shutdown)
        self.get_logger().info(f"Listener node started. Will print {self.max_messages} odometry messages or shutdown after {self.shutdown_timer_duration} seconds.")

    def odom_callback(self, msg):
        self.received_messages += 1
        self.get_logger().info(
            f"Received odometry ({self.received_messages}/{self.max_messages}):\n"
            f"  Pose: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f}\n"
            f"  Orientation (quaternion): w={msg.pose.pose.orientation.w:.2f}, x={msg.pose.pose.orientation.x:.2f}, y={msg.pose.pose.orientation.y:.2f}, z={msg.pose.pose.orientation.z:.2f}\n"
            f"  Twist: linear.x={msg.twist.twist.linear.x:.2f}, angular.z={msg.twist.twist.angular.z:.2f}"
        )
        if self.received_messages >= self.max_messages:
            self.get_logger().info(f"Received {self.max_messages} messages. Shutting down listener node.")
            if self.shutdown_timer:
                self.shutdown_timer.cancel()
            rclpy.shutdown()

    def timed_shutdown(self):
        if rclpy.ok(): # Check if not already shut down by callback
            self.get_logger().info(f"Shutdown timer of {self.shutdown_timer_duration} seconds elapsed. Shutting down listener node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    try:
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        listener_node.get_logger().info('Keyboard interrupt received. Shutting down listener node.')
    finally:
        # This is important to ensure resources are released, especially if shutdown
        # was initiated from within the node or due to an exception.
        if rclpy.ok(): # Check if shutdown was already called
            listener_node.get_logger().info('Listener node shutting down from main.')
        listener_node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown() # Attempt to shutdown ROS context if not already done

if __name__ == '__main__':
    main()
