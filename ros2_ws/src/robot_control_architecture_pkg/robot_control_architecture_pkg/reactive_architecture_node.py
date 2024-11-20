import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class ReactiveRobotControlNode(Node):
    def __init__(self):
        print("Initializing node...")
        super().__init__('reactive_robot_control_node')

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

        self.mobilenet_sub = self.create_subscription(
            String,
            '/color/mobilenet_detections',
            self.detection_callback,
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
            # celebration
            twist.angular.z = self.ANGULAR_SPEED
            print(f"Congratulations, your high score is {self.score}")

        elif self.state == self.COLLECT_COIN:
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
