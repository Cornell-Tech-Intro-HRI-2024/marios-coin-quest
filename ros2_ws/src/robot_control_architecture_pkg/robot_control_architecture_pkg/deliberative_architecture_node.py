import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import math

# Import odometry
from nav_msgs.msg import Odometry
import cv2

class DeliberativeArchitectureNode(Node):
    def __init__(self):
        print("Initializing node...")
        super().__init__('deliberative_architecture_node')

        self.NAVIGATE = 0
        self.STOP = 1
        self.OUT_OF_BOUNDS = 2
        self.state = self.NAVIGATE
        
        # Odometry
        self.current_pose = None
        self.starting_pose = None
        self.target_pose = None

        self.LINEAR_SPEED = 0.3
        self.ANGULAR_SPEED = 0.5
        self.OBSTACLE_THRESHOLD = 0.5

        self.br = CvBridge()
        self.score = 0 # game score
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10))

        # publisher
        self.mario_pub = self.create_publisher(
            String,
            '/mario_status',
            QoSProfile(depth=10)
        )
        
        # List of coin positions
        
        
        self.timer = self.create_timer(0.1, self.control_cycle)
        print("Finished initialization")


    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw (z-axis rotation)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_pose = (position.x, position.y, self.quaternion_to_yaw(orientation))
        # Use the first odom callback to set the starting pose for the robot
        if self.starting_pose is None:
            self.starting_pose = self.current_pose
            # Arbitrary target location relative to the start for testing
            self.target_pose = (position.x + 0.8, position.y, self.quaternion_to_yaw(orientation))
            self.set_starting_positions()
            
    def set_starting_positions(self):
        # Define the "out-of-bounds" limits
        self.minX = self.starting_pose[0] - 1
        self.minY = self.starting_pose[1] - 1
        self.maxX = self.starting_pose[0] + 1
        self.maxY = self.starting_pose[1] + 1
        
    def is_out_of_bounds(self):
        pos = self.current_pose
        if self.minX < pos[0] < self.maxX and self.minY < pos[1] < self.maxY:
            return False
        else:
            return True

    def control_cycle(self):
        # If odometry is not initialized yet, don't run anything
        if self.starting_pose is None:
            print("Waiting for odometry...")
            return

        if self.state == self.NAVIGATE:
            pos = self.current_pose
            x = pos[0]
            y = pos[1]
            z = pos[2]
            print("Current pose is ", f"{x:.3g}", ", ", f"{y:.3g}", ", ", f"{z:.3g}")
            #twist.linear.x = self.LINEAR_SPEED
            #twist.linear.x = 0.0
            if self.is_out_of_bounds():
                msg = String()
                msg.data = "OUT_OF_BOUNDS"
                self.mario_pub.publish(msg)
                self.state = self.OUT_OF_BOUNDS
                print("Out of bounds!")
        
        elif self.state == self.OUT_OF_BOUNDS:
            if not self.is_out_of_bounds():
                msg = String()
                msg.data = "IN_BOUNDS"
                self.mario_pub.publish(msg)
                self.state = self.NAVIGATE
                print("Back in bounds!")
        
def main(args=None):
    rclpy.init(args=args)

    robot_control_node = DeliberativeArchitectureNode()
    rclpy.spin(robot_control_node)
    print('node running...')

    robot_control_node.destroy_node()
    print('destroying node...')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

