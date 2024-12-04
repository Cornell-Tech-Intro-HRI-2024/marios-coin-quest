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
        
        # Initialize starting positions
        self.set_starting_positions()
        
        self.timer = self.create_timer(0.1, self.control_cycle)
        print("Finished initialization")


    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw (z-axis rotation)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Handle incoming odometry messages and compute local space transform"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert quaternion orientation to yaw
        yaw = self.quaternion_to_yaw(orientation)

        # Record the initial pose (local space origin)
        if self.starting_pose is None:
            self.starting_pose = (position.x - 0.1, position.y + 0.1, yaw)
        
        # Transform odometry into local space
        dx = position.x - self.starting_pose[0]
        dy = position.y - self.starting_pose[1]
        dtheta = yaw - self.starting_pose[2]

        # Rotate into local frame (account for initial yaw)
        local_x = dx * math.cos(-self.starting_pose[2]) - dy * math.sin(-self.starting_pose[2])
        local_y = dx * math.sin(-self.starting_pose[2]) + dy * math.cos(-self.starting_pose[2])

        # Store the transformed pose
        self.current_pose = (local_x, local_y, dtheta)
            
    def set_starting_positions(self):
        # Define the "out-of-bounds" limits
        self.minX = -0.2
        self.minY = -2.8
        self.maxX = 2.8
        self.maxY = 0.2
        
        # Define the "coin" locations
        self.coins = [(2.0, -0.6), (2.6, -1.5), (0.8, -2.0), (0.7, -2.25), (2.2, -2.5)]
        
    def is_out_of_bounds(self):
        pos = self.current_pose
        if self.minX < pos[0] < self.maxX and self.minY < pos[1] < self.maxY:
            return False
        else:
            return True
    
    def check_for_coins(self):
        """
        Check if the character's current position is within 0.1 of any coin.
        If so, trigger a coin collection and remove the coin from the list.
        """
        collected_coins = []

        for coin in self.coins:
            coin_x, coin_y = coin
            x, y, _ = self.current_pose  # Unpack current position (x, y, yaw)

            # Calculate distance to the coin
            distance = ((coin_x - x) ** 2 + (coin_y - y) ** 2) ** 0.5

            # Check if within collection range (0.1)
            if distance <= 0.1:
                # Send a message indicating a coin was collected
                msg = String()
                msg.data = "COIN_COLLECTED"
                print("Coin collected!")
                self.mario_pub.publish(msg)
                collected_coins.append(coin)  # Mark coin for removal

        # Remove collected coins from the list
        for coin in collected_coins:
            self.coins.remove(coin)
            
    def check_for_flag():
        x, y, _ = self.current_pose  # Unpack current position (x, y, yaw)
        flag_x, flag_y = (2.6, 0)
        
        # Calculate distance to the flag
        distance = ((flag_x - x) ** 2 + (flag - y) ** 2) ** 0.5
        
        # Check if within range (0.3)
        if distance <= 0.3:
            # Send a message indicating a coin was collected
            msg = String()
            msg.data = "REACHED_FLAG"
            print("Reached flag!")
            self.mario_pub.publish(msg)
            self.state = self.STOP

    def control_cycle(self):
        # If odometry is not initialized yet, don't run anything
        if self.starting_pose is None:
            print("Waiting for odometry...")
            return
        
        if self.state == self.NAVIGATE:
            print(self.current_pose)
            pos = self.current_pose
            x = pos[0]
            y = pos[1]
            z = pos[2]
            #print("Current pose is ", f"{x:.3g}", ", ", f"{y:.3g}", ", ", f"{z:.3g}")
            #twist.linear.x = self.LINEAR_SPEED
            #twist.linear.x = 0.0
            if self.is_out_of_bounds():
                msg = String()
                msg.data = "OUT_OF_BOUNDS"
                self.mario_pub.publish(msg)
                self.state = self.OUT_OF_BOUNDS
                print("Out of bounds!")
            
            # Check for coins
            self.check_for_coins()
        
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
