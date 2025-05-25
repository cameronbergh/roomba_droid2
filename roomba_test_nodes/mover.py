import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoverNode(Node):
    def __init__(self):
        super().__init__('roomba_mover_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.iterations = 0
        self.max_iterations = 10 # Publish for 1 second (10 * 0.1s)

    def publish_once_and_shutdown(self):
        msg = Twist()
        msg.linear.x = 0.1  # Move forward at 0.1 m/s
        msg.angular.z = 0.0 # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Twist: linear.x=0.1, angular.z=0.0')
        
        # Wait for a moment to ensure the message is sent before shutting down
        # This is a simple way for a short-lived publisher.
        # For more robust publishing, one might wait for subscriber acknowledgments
        # or use a timer for repeated publishing.
        time.sleep(0.5) 
        self.get_logger().info('Twist message published. Shutting down mover node.')
        rclpy.shutdown()

    def timed_publish(self):
        if self.iterations < self.max_iterations:
            msg = Twist()
            msg.linear.x = 0.1  # Move forward at 0.1 m/s
            msg.angular.z = 0.0 # No rotation
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing Twist (iteration {self.iterations+1}/{self.max_iterations}): linear.x=0.1, angular.z=0.0')
            self.iterations += 1
        else:
            # Stop the robot
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Max iterations reached. Stopping robot and shutting down mover node.')
            # Give a moment for the stop message to be sent
            time.sleep(0.5)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    mover_node = MoverNode()

    # Option 1: Publish once and shutdown
    # mover_node.publish_once_and_shutdown()

    # Option 2: Publish for a short duration using a timer
    # For this option, we'll manually spin the node until shutdown is called from the timer callback
    timer = mover_node.create_timer(mover_node.timer_period, mover_node.timed_publish)
    
    try:
        rclpy.spin(mover_node)
    except KeyboardInterrupt:
        pass # Allow Ctrl+C to interrupt
    finally:
        # Ensure the node is destroyed cleanly if spin exits due to an exception
        # or if shutdown was called from within the node.
        if rclpy.ok(): # Check if shutdown was already called
             mover_node.get_logger().info('Mover node shutting down from main.')
             # Publish a final stop command if interrupted early
             stop_msg = Twist()
             stop_msg.linear.x = 0.0
             stop_msg.angular.z = 0.0
             mover_node.publisher_.publish(stop_msg)
             time.sleep(0.1) # Brief pause for the message
        mover_node.destroy_node()
        if rclpy.ok(): # Check if shutdown was already called by the node itself
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
