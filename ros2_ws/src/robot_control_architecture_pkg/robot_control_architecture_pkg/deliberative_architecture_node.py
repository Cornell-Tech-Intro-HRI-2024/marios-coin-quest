import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Import odometry
from nav_msgs.msg import Odometry
import cv2

class DeliberativeArchitectureNode(Node):
    def __init__(self):
        print("Initializing node...")
        super().__init__('deliberative_architecture_node')

        self.FORWARD = 0
        self.STOP = 1
        self.state = self.FORWARD
        
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
            10)

        # publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)


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
            self.target_pose = (position.x + 3.0, position.y, self.quaternion_to_yaw(orientation))
        

    def control_cycle(self):
        if not self.last_scan or not self.last_rgb_image or not self.last_depth_image:
            return

        twist = Twist()

        if self.state == self.NAVIGATE:
            print("Target pose is ", self.target_pose, " and current pose is ", self.current_pose)
            #twist.linear.x = self.LINEAR_SPEED

        elif self.state == self.STOP:
            #twist.linear.x = 0.0
        
        

        #self.vel_pub.publish(twist)
