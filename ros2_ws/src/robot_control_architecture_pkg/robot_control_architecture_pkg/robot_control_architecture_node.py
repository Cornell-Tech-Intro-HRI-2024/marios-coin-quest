import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from vision_msgs.msg import Detection2DArray
import cv2
from cv_bridge import CvBridge

class RobotControlArchitectureNode(Node):
    def __init__(self):
        print("Initiliasing node...")
        super().__init__('robot_control_architecture_node')

        self.SEARCH = 0
        self.FORWARD = 1
        self.AVOID_TURN = 2
        self.AVOID_FORWARD = 3
        self.END = 4
        self.state = self.SEARCH
        self.state_ts = self.get_clock().now()
        self.avoid_time = Duration(seconds=0)
        
        print("Initializing parameters...")
        # Parameters
        self.TURNING_SPEED = 0.6 # Positive = turn left
        self.LINEAR_SPEED = 0.5
        self.OBSTACLE_DISTANCE = 0.8
        
        self.TIME_TILL_PERSON = 4.0 # Amount of time the robot will travel forward before assuming it has reached the person
        self.AVOID_FORWARD_TIME = 2.5 # Amount of time the robot will move to avoid an obstacle before going back to searching
        self.AVOID_TURN_TIME = 1.0 # Amount of time the robot will back up (& turn) to avoid an obstacle

        self.br = CvBridge()
        self.last_scan = None
        self.last_rgb_image = None
        self.person_detected = False
        self.person_center_x = 0
        self.obstacle_avoided = False
        
        print("Initializing subscriptions...")
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)

        # The depth subscriber has been omitted due to the depth publisher being non-functional.
        # self.depth_sub = self.create_subscription(
        #     Image, '/stereo/depth', self.depth_callback, 10)
        
        self.rgb_sub = self.create_subscription(
            Image, '/color/image', self.rgb_callback, 10)
        
        self.person_detection_sub = self.create_subscription(
            Detection2DArray, 'color/mobilenet_detections', self.person_detection_callback, 10)
        
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_cycle)
        print("Finished initialization")

    def scan_callback(self, msg):
        self.last_scan = msg

    # The depth callback function has been omitted due to the depth publisher being non-functional.
    # def depth_callback(self, msg):
    #         try:
    #            self.last_depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #         except cv2.error as e:
    #             self.get_logger().error(f"Failed to convert depth image {e}")
    #             return
    
    def rgb_callback(self, msg):
            try:
                self.last_rgb_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except cv2.error as e:
                self.get_logger().error(f"Failed to convert RGB image {e}")
                return

    def person_detection_callback(self, msg): # Heavily modified based on a code snippet provided by Angelique
        '''
        Input: YOLO person detection
        '''
        self.person_detections = msg
        # List of detection: msg.detections
        for det in msg.detections:
            hypothesis = det.results[0].hypothesis
            # The class ID of 15 indicates that the object detected is a person; score is a confidence threshold
            # If the detected object is a person with >80% confidence, proceed
            if hypothesis.class_id == "15" and hypothesis.score > 0.7:
                self.person_detected = True
                # Save the horizontal position of the person in the camera view, so we can turn to face them.
                self.person_center_x = det.bbox.center.position.x

    def control_cycle(self):
        if not self.last_scan:
            print("Waiting for scan data...")
            return

        # Test code for debugging obstacle detection; only prints LIDAR output and does nothing else
        # print(self.is_obstacle_ahead())
        # print(self.last_scan.ranges[265:275])
        # return

        out_vel = Twist()

        if self.state == self.SEARCH:
            # By default, turn to the left
            out_vel.angular.z = self.TURNING_SPEED
            # If we see someone on the right side of the camera, turn to the right instead
            if self.person_detected and self.person_center_x > 200:
                out_vel.angular.z = -self.TURNING_SPEED

            # Transition to FORWARD if a person is in the center of the view
            if self.check_search_2_forward():
                self.go_state(self.FORWARD)

        elif self.state == self.FORWARD:
            # Move forward
            out_vel.linear.x = self.LINEAR_SPEED
            # If more than TIME_TILL_PERSON seconds have passed since we started moving forward, and we have not run into an obstacle yet,
            # assume we have reached the person; transition to STOP and finalize movement
            if self.check_forward_2_end():
                self.go_state(self.END)
            # If there is an obstacle before we reach that limit, assume it is an object to be avoided; transition into AVOID_TURN 
            if self.check_forward_2_avoid_turn():
                self.go_state(self.AVOID_TURN)

        elif self.state == self.AVOID_TURN:
            # Turn to the right
            out_vel.angular.z = -self.TURNING_SPEED
            # Once there is no longer an obstacle directly in front of us, start the countdown to leaving this state
            # This ensures that we only turn as much as we need to avoid something - 
            # less for small objects, more for larger objects.
            if not self.obstacle_avoided and min(self.last_scan.ranges[250:290]) > 2 * self.OBSTACLE_DISTANCE:
                self.obstacle_avoided = True
                self.state_ts = self.get_clock().now()

            # A short time after clearing the obstacle, transition to AVOID_FORWARD
            if self.check_avoid_turn_2_avoid_forward():
                self.go_state(self.AVOID_FORWARD)
                # Reset the variable for next time
                self.obstacle_avoided = False

        elif self.state == self.AVOID_FORWARD:
            # Move forward; since we have turned to the right, we are moving around the right side of the object
            out_vel.linear.x = self.LINEAR_SPEED
            # After enough time moving forward, go back to search
            if self.check_avoid_forward_2_search():
                # Reset person detection
                self.person_detected = False
                # Restart person search
                self.go_state(self.SEARCH)

        elif self.state == self.END:
            # Do nothing once we've finished
            x = 1

        self.vel_pub.publish(out_vel)

    # True when a person is detected and in the center of the view.
    def check_search_2_forward(self):
        return self.person_detected and self.person_center_x > 100 and self.person_center_x < 200

    # True when an obstacle is in front of the robot.
    def check_forward_2_avoid_turn(self):
        return self.is_obstacle_ahead()

    # True when TIME_TILL_PERSON seconds have passed without encountering an obstacle.
    def check_forward_2_end(self):
        return self.elapsed_time_in_state() > Duration(seconds=self.TIME_TILL_PERSON)

    # True when AVOID_TURN_TIME seconds have passed after avoiding an obstacle.
    def check_avoid_turn_2_avoid_forward(self):
        return self.obstacle_avoided and self.elapsed_time_in_state() > Duration(seconds=self.AVOID_TURN_TIME)

    # True then AVOID_FORWARD_TIME seconds have passed.
    def check_avoid_forward_2_search(self):
        return self.elapsed_time_in_state() > Duration(seconds=self.AVOID_FORWARD_TIME)
    
    def go_state(self, new_state):
        self.state = new_state
        
        if self.state == self.SEARCH:
            print("Entering SEARCH")
        elif self.state == self.FORWARD:
            print("Entering FORWARD")
        elif self.state == self.AVOID_TURN:
            print("Entering AVOID_TURN")
        elif self.state == self.AVOID_FORWARD:
            print("Entering AVOID_FORWARD")
        elif self.state == self.END:
            print("Entering END")
            
        self.state_ts = self.get_clock().now()

    def is_obstacle_ahead(self):
        if self.last_scan:
            # For reasons unknown to god or man, the 0th index in this list (out of 360) corresponds to the distance on the right side.
            # Take the distance reading at 90 degrees to get the scan directly in front of the robot.
            #pos = len(self.last_scan.ranges) // 2 # This is the old code, which looked at 180 - the left side.
            # pos = len(self.last_scan.ranges) // 4
            print("Range is ", self.last_scan.ranges[270])
            return min(self.last_scan.ranges[250:290]) < self.OBSTACLE_DISTANCE

        print ("No last scan found")
        return False

    def elapsed_time_in_state(self):
        return self.get_clock().now() - self.state_ts

def main(args=None):
    rclpy.init(args=args)

    robot_control_node = RobotControlArchitectureNode()
    rclpy.spin(robot_control_node)
    print('node running...')

    robot_control_node.destroy_node()
    print('destroying node...')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
