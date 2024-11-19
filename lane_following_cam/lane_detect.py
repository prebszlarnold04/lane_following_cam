import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from rclpy.node import Node
import rclpy

class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')
        self.start()
        self.setup_subscribers_and_publishers()
        self.initialize_pid_parameters()
        self.bridge = CvBridge()

    def start(self):
        self.declare_parameter('raw_image', False)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug', True)

    def setup_subscribers_and_publishers(self):
        img_topic = self.get_parameter('image_topic').value
        if self.get_parameter('raw_image').value:
            self.sub1 = self.create_subscription(Image, img_topic, self.raw_listener, 10)
            self.get_logger().info(f'lane_detect subscribed to raw image topic: {img_topic}')
        else:
            self.sub2 = self.create_subscription(CompressedImage, '/image_raw/compressed', self.compr_listener, 10)
            self.get_logger().info(f'lane_detect subscribed to compressed image topic: {img_topic}')
        self.pub1 = self.create_publisher(Image, '/lane_img', 10)
        self.pub2 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.offset_pub = self.create_publisher(Float32, '/lane_center_offset', 10)

    def initialize_pid_parameters(self):
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0.05
        self.max_angle = 0.4  # Max steering angle in radians
        self.speed_mps = 0.5  # Speed in meters per second
        self.t_previous = self.get_clock().now()
        self.e_previous = 0
        self.P, self.I, self.D = 0, 0, 0

    def raw_listener(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.get_logger().info(f'First raw img arrived, shape: {cv_image.shape}', once=True)
        lane_image = self.detect_lanes(cv_image)
        ros_image = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        self.pub1.publish(ros_image)
    
    def compr_listener(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.get_logger().info(f'First compressed img arrived, shape: {cv_image.shape}', once=True)
        lane_image = self.detect_lanes(cv_image)
        ros_image = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        self.pub1.publish(ros_image)

    def detect_lanes(self, image):
        imageBrighnessHigh = self.increase_brightness(image)
        edges = self.apply_edge_detection(imageBrighnessHigh)
        left_fitx, right_fitx, ploty, midpoint = self.sliding_window(edges)
        center_deviation = self.calculate_center_deviation(left_fitx, right_fitx, midpoint)
        self.publish_center_offset(center_deviation)
        self.publish_twist_message(center_deviation)
        return self.visualize_lanes(imageBrighnessHigh, left_fitx, right_fitx, ploty, center_deviation)

    def increase_brightness(self, image):
        return cv2.convertScaleAbs(image, alpha=1, beta=10)

    def apply_edge_detection(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.convertScaleAbs(gray, alpha=1, beta=10) 
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(gray, 100, 200)

    def sliding_window(self, edges):
        window_height = 40
        num_windows = 10
        margin = 50
        minpix = 50

        histogram = np.sum(edges[edges.shape[0]//2:,:], axis=0)
        midpoint = int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nonzero = edges.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        left_lane_inds = []
        right_lane_inds = []

        for window in range(num_windows):
            win_y_low = edges.shape[0] - (window + 1) * window_height
            win_y_high = edges.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = np.array(nonzero[0])
        rightx = nonzerox[right_lane_inds]
        righty = np.array(nonzero[0])

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, edges.shape[0] - 1, edges.shape[0])
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

        return left_fitx, right_fitx, ploty, midpoint

    def calculate_center_deviation(self, left_fitx, right_fitx, midpoint):
        center_fitx = (left_fitx + right_fitx) / 2
        return center_fitx[-1] - midpoint

    def publish_center_offset(self, center_deviation):
        self.offset_pub.publish(Float32(data=center_deviation))

    def publish_twist_message(self, center_deviation):
        twist = Twist()
        twist.linear.x = 0.2  # Constant forward speed
        twist.angular.z = -0.01 * center_deviation  # Adjust angular speed based on deviation
        self.pub2.publish(twist)

    def visualize_lanes(self, image, left_fitx, right_fitx, ploty, center_deviation):
        line_image = np.zeros_like(image)
        for i in range(len(ploty)):
            cv2.circle(line_image, (int(left_fitx[i]), int(ploty[i])), 2, (255, 0, 0), -1)
            cv2.circle(line_image, (int(right_fitx[i]), int(ploty[i])), 2, (0, 255, 0), -1)
            cv2.circle(line_image, (int((left_fitx[i] + right_fitx[i]) / 2), int(ploty[i])), 2, (0, 0, 255), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        direction = 'Straight'
        if center_deviation > 0.01:
            direction = 'Right'
        elif center_deviation < -0.01:
            direction = 'Left'
        cv2.putText(line_image, f'{direction} {abs(center_deviation):.2f}', (10, 30), font, 1, (60, 40, 200), 2, cv2.LINE_AA)

        if self.debug:
            return line_image
        else:
            return cv2.addWeighted(image, 0.8, line_image, 1, 1)

    def pid_control(self, error):
        t = self.get_clock().now()
        dt = (t - self.t_previous).nanoseconds / 1e9
        de = self.e_previous - error 
        self.P = error
        self.I = self.I + error * dt
        self.D = de / dt if dt > 0 else 0
        steer_rad = self.kp * self.P + self.ki * self.I + self.kd * self.D
        
        steer_rad = max(-self.max_angle, min(steer_rad, self.max_angle))
        
        self.t_previous = t
        self.e_previous = error

        return steer_rad

    def callback(self, data):
        error = data.data
        steer_rad = self.pid_control(error)

        twist = Twist()
        twist.linear.x = self.speed_mps
        twist.angular.z = steer_rad

        self.pub2.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    lane_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()