import rclpy
from geometry_msgs.msg import Twist
import numpy as np

class MinimalVelPubSubNode:
    def __init__(self):
        rclpy.init()

        self.node = rclpy.create_node('minimal_vel_pub_sub')
        self.subscription = self.node.create_subscription(Twist, '/cmd_vel', self.chatter_callback, 10)
        self.publisher = self.node.create_publisher(Twist, 'motor_velocity', 10)

    def chatter_callback(self, msg):
        try:
            # Extract linear x, linear y, and angular z velocities
            vx = msg.linear.x
            vy = msg.linear.y
            vtheta = msg.angular.z

            # Calculate motor speeds
            robot_frame_speed = np.array([[vx], [vy], [vtheta]], dtype=float)
            alpha = np.array([np.deg2rad(35.77), np.deg2rad(144.23), np.deg2rad(215.77), np.deg2rad(324.23)], dtype=float)
            beta = np.array([np.deg2rad(144.23), np.deg2rad(35.77), np.deg2rad(-35.77), np.deg2rad(-144.23)], dtype=float)
            gamma = np.array([np.deg2rad(45), np.deg2rad(-45), np.deg2rad(45), np.deg2rad(-45)], dtype=float)
            r = 0.0325
            L = 0.0937
            rolling_constraints = np.array([
                [np.sin(alpha[0] + beta[0] + gamma[0]), -np.cos(alpha[0] + beta[0] + gamma[0]), -L * np.cos(beta[0] + gamma[0])],
                [np.sin(alpha[1] + beta[1] + gamma[1]), -np.cos(alpha[1] + beta[1] + gamma[1]), -L * np.cos(beta[1] + gamma[1])],
                [np.sin(alpha[2] + beta[2] + gamma[2]), -np.cos(alpha[2] + beta[2] + gamma[2]), -L * np.cos(beta[2] + gamma[2])],
                [np.sin(alpha[3] + beta[3] + gamma[3]), -np.cos(alpha[3] + beta[3] + gamma[3]), -L * np.cos(beta[3] + gamma[3])],
            ], dtype=float)
            radius_matrix = np.array([
                [r * np.cos(gamma[0]), 0, 0, 0],
                [0, r * np.cos(gamma[1]), 0, 0],
                [0, 0, r * np.cos(gamma[2]), 0],
                [0, 0, 0, r * np.cos(gamma[3])]
            ], dtype=float)
            motor_speeds = np.linalg.inv(radius_matrix) @ rolling_constraints @ robot_frame_speed

            # Create and publish Twist message with calculated motor speeds
            motor_twist = Twist()
            motor_twist.linear.x = motor_speeds[0, 0]
            motor_twist.linear.y = motor_speeds[1, 0]
            motor_twist.linear.z = motor_speeds[2, 0]
            motor_twist.angular.z = motor_speeds[3, 0]

            self.publisher.publish(motor_twist)

            self.node.get_logger().info('Published Motor Speeds: %s' % motor_twist)
        except Exception as e:
            self.node.get_logger().warn('Error processing cmd_vel message: %s' % str(e))

    def spin(self):
        rclpy.spin(self.node)

    def destroy_node(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    node = MinimalVelPubSubNode()
    node.spin()
    node.destroy_node()

if __name__ == '__main__':
    main()
