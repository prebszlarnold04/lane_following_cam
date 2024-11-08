import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')
        # parameters
        self.declare_parameter('raw_image', False)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('debug', True)
        img_topic = self.get_parameter('image_topic').value
        if self.get_parameter('raw_image').value:
            self.sub1 = self.create_subscription(Image, img_topic, self.raw_listener, 10)
            self.sub1  # prevent unused variable warning
            self.get_logger().info(f'lane_detect subscribed to raw image topic: {img_topic}')
        else:
            self.sub2 = self.create_subscription(CompressedImage, '/image_raw/compressed', self.compr_listener, 10)
            self.sub2  # prevent unused variable warning
            self.get_logger().info(f'lane_detect subscribed to compressed image topic: {img_topic}')
        self.pub1 = self.create_publisher(Image, '/lane_img', 10)
        self.pub2 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.debug = self.get_parameter('debug').value

    def raw_listener(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # print info of the image, only once not every time
        self.get_logger().info(f'First raw img arrived, shape: {cv_image.shape}', once=True)
        # Detect lanes
        lane_image = self.detect_lanes(cv_image)
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        # Publish the image
        self.pub1.publish(ros_image)
    
    def compr_listener(self, msg):
        # Convert ROS CompressedImage message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # print info of the image, only once not every time
        self.get_logger().info(f'First compressed img arrived, shape: {cv_image.shape}', once=True)
        # Detect lanes
        lane_image = self.detect_lanes(cv_image)
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(lane_image, 'bgr8')
        # Publish the image
        self.pub1.publish(ros_image)

    def detect_lanes(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Detect edges using Canny
        edges = cv2.Canny(blur, 50, 150)
        # Define a region of interest
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height * 0.6)),
            (0, int(height * 0.6))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, maxLineGap=50)
        line_image = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (60, 200, 20), 5)
        center = width / 2
        twist = Twist()
        # Find the center of the lines and steer the robot
        if lines is not None:
            left_x = []
            right_x = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 < width / 2 and x2 < width / 2:
                    left_x.append(x1)
                    left_x.append(x2)
                elif x1 > width / 2 and x2 > width / 2:
                    right_x.append(x1)
                    right_x.append(x2)
            sum_left = sum(left_x)
            sum_right = sum(right_x)
            # Ratio of the sum of the x-coordinates of the left lines to the right lines
            if len(left_x) > 0 and len(right_x) > 0:
                ratio = sum_left / sum_right
                twist.linear.x = 0.2
                twist.angular.z = 0.1 * (1 - ratio)
                self.pub2.publish(twist)
        else:
            # If there are no lines detected, slow the robot
            twist.angular.z = 0.0
            twist.linear.x = 0.05
            self.pub2.publish(twist)

        # Display red point at the center of the image, and move based on twist.angular.z 
        cv2.circle(line_image, (int(center + int(twist.angular.z * width * 5) ), int(height / 2.)), 5, (60, 40, 200), -1)

        # Display the twist.angular.z value on the image and direction (left or right or straight)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if twist.angular.z > 0.01:
            text = 'Right'
        elif twist.angular.z < -0.01:
            text = 'Left'
        else:
            text = 'Straight'
        cv2.putText(line_image, f'{text} {abs(twist.angular.z):.2f}', (10, 30), font, 1, (60, 40, 200), 2, cv2.LINE_AA)


        # Combine the original image with the line image
        if self.debug:
            combined_image = line_image
        else: 
            combined_image = cv2.addWeighted(image, 0.8, line_image, 1, 1)
        return combined_image

def main(args=None):
    rclpy.init(args=args)
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    lane_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
