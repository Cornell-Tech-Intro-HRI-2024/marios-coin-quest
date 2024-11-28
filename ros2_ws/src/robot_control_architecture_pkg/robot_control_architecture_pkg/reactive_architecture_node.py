import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import math
import time

class ReactiveArchitectureNode(Node):
    def __init__(self):
        print("Initializing node...")
        super().__init__('reactive_architecture_node')

        self.NAVIGATE = 0
        self.TURN_LEFT = 1
        self.TURN_RIGHT = 2
        self.STOP = 3
        self.CELEBRATE = 4
        self.COLLECT_COIN = 5
        self.state = self.NAVIGATE

        self.LINEAR_SPEED = 0.3
        self.ANGULAR_SPEED = 0.5
        self.OBSTACLE_THRESHOLD = 0.5

        self.br = CvBridge()
        self.score = 0 # game score

        self.coin_locations = [(1.0, 2.0), (3.5, 1.2)] #dummy coordinates where coins are placed
        self.coin_tolerance = 0.1 #coordinate tolerance for detecting coin collection

        self.current_pose = None
        self.celebrate_start_time = None #track celebration time

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.rgb_sub = self.create_subscription(
            Image,
            '/color/image', 
            self.rgb_callback, 10)

        self.depth_sub = self.create_subscription(
            Image,
            '/stereo/depth', 
            self.depth_callback, 10)

        self.detection_sub = self.create_subscription(
            String,
            '/color/mobilenet_detections',
            self.detection_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # control loop timer
        self.timer = self.create_timer(0.1, self.control_cycle)

        self.last_scan = None
        self.last_rgb_image = None
        self.last_depth_image = None
        self.detection_message = None

    def scan_callback(self, msg):
        self.last_scan = msg

    def rgb_callback(self, msg):
        try:
            self.last_rgb_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv2.error as e:
            self.get_logger().error(f"Failed to convert RGB image {e}")

    def depth_callback(self, msg):
        try:
            self.last_depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except cv2.error as e:
            self.get_logger().error(f"Failed to convert Depth image {e}")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_pose = (position.x, position.y)

    def detection_callback(self, msg):
        self.detection_message = msg.data

    def control_cycle(self):
        if not self.last_scan or not self.last_rgb_image or not self.last_depth_image:
            return

        twist = Twist()

        if self.state == self.NAVIGATE:
            twist.linear.x = self.LINEAR_SPEED
            if self.is_obstacle_ahead():
                if self.detect_red_obstacle():
                    self.state = self.TURN_RIGHT
                elif self.detect_green_obstacle():
                    self.state = self.TURN_LEFT
            elif self.detect_flag():
                self.state = self.CELEBRATE
            elif self.detect_coin():
                self.state = self.COLLECT_COIN
            elif self.detect_boundary():
                self.state = self.STOP

        elif self.state == self.TURN_LEFT:
            twist.angular.z = self.ANGULAR_SPEED
            if not self.is_obstacle_ahead():
                self.state = self.NAVIGATE

        elif self.state == self.TURN_RIGHT:
            twist.angular.z = -self.ANGULAR_SPEED
            if not self.is_obstacle_ahead():
                self.state = self.NAVIGATE

        elif self.state == self.STOP:
            twist.linear.x = 0
            twist.angular.z = 0
            print(f"Score: {self.score}")

        elif self.state == self.CELEBRATE:
            twist.angular.z = self.ANGULAR_SPEED
            print(f"Yay, we did it! Your high score is {self.score}")
            if time.time() - self.celebrate_start_time >= 5.0: #if celebration is longer than 5 seconds
                twist.linear.x = 0
                twist.angular.z = 0
                self.state = self.STOP

        elif self.state == self.COLLECT_COIN:
            if self.is_coin_within_reach():
                self.score += 500
                print(f"New score: {self.score}")
            self.state = self.NAVIGATE

        self.vel_pub.publish(twist)

    def is_obstacle_ahead(self):
        return min(self.last_scan.ranges) < self.OBSTACLE_THRESHOLD

    def detect_green_obstacle(self):
        # analyze RGB image to detect green obstacles
        hsv_image = cv2.cvtColor(self.last_rgb_image, cv2.COLOR_BGR2HSV)
        lower_green = (36, 100, 100)
        upper_green = (86, 255, 255)
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        return cv2.countNonZero(mask) > 1000

    def detect_red_obstacle(self):
        # analyze RGB image to detect red obstacles
        hsv_image = cv2.cvtColor(self.last_rgb_image, cv2.COLOR_BGR2HSV)
        lower_red1 = (0, 120, 70)
        upper_red1 = (10, 255, 255)
        lower_red2 = (170, 120, 70)
        upper_red2 = (180, 255, 255)
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        return (cv2.countNonZero(mask1) + cv2.countNonZero(mask2)) > 1000

    def detect_flag(self):
        # analyze RGB image to detect black flag
        hsv_image = cv2.cvtColor(self.last_rgb_image, cv2.COLOR_BGR2HSV)
        lower_black = (0, 0, 0)
        upper_black = (180, 255, 30)
        mask = cv2.inRange(hsv_image, lower_black, upper_black)
        reached_flag = cv2.countNonZero(mask) > 1000
        if reached_flag:
            self.score += 1500
        return reached_flag

    def detect_boundary(self):
        # LIDAR check for boundaries
        min_distance = min(self.last_scan.ranges)
        return min_distance < self.OBSTACLE_THRESHOLD
    
    def detect_coin(self):
        # check if coin is detected
        return "coin" in self.detection_message if self.detection_message else False
    
    def is_coin_within_reach(self):
        if not self.current_pose:
            return False
        for coin_location in self.coin_locations:
            if self.euclidean_distance(self.current_pose, coin_location) < self.coin_tolerance:
                return True
        return False

    def euclidean_distance(self, pose, target):
        return math.sqrt((pose[0] - target[0])**2 + (pose[1] - target[1])**2)
    

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveArchitectureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
