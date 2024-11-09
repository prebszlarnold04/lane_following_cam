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
        # Step 1: Convert to grayscale and apply Canny edge detection and Gaussian blur
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 100, 200)
        
        # Step 2: Crop a narrow stripe in front of the vehicle
        ymin, ymax = 200, 240  # Define based on your camera view
        stripe = edges[ymin:ymax, :]

        # Step 3: Find left and right lane borders within the stripe
        center_x = stripe.shape[1] // 2
        cx_left = np.mean(np.nonzero(stripe[:, :center_x])[1]) if np.any(stripe[:, :center_x]) else center_x
        cx_right = np.mean(np.nonzero(stripe[:, center_x:])[1]) + center_x if np.any(stripe[:, center_x:]) else center_x

        # Step 4: Calculate the road center and adjust steering
        cx_road = (cx_left + cx_right) / 2
        center_deviation = cx_road - center_x

        # Prepare Twist message for steering
        twist = Twist()
        twist.linear.x = 0.2  # Constant forward speed
        twist.angular.z = -0.01 * center_deviation  # Adjust angular speed based on deviation
        self.pub2.publish(twist)

        # Debug: Visualize the lane and center points
        line_image = np.zeros_like(image)
        cv2.line(line_image, (int(cx_left), ymin), (int(cx_left), ymax), (255, 0, 0), 2)  # Left lane border
        cv2.line(line_image, (int(cx_right), ymin), (int(cx_right), ymax), (0, 255, 0), 2)  # Right lane border
        cv2.circle(line_image, (int(cx_road), (ymin + ymax) // 2), 5, (0, 0, 255), -1)  # Center point

        # Display the twist.angular.z value on the image and direction (left, right, or straight)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if twist.angular.z > 0.01:
            text = 'Right'
        elif twist.angular.z < -0.01:
            text = 'Left'
        else:
            text = 'Straight'
        cv2.putText(line_image, f'{text} {abs(twist.angular.z):.2f}', (10, 30), font, 1, (60, 40, 200), 2, cv2.LINE_AA)

        
        # Combine the original image with the line image for visualization
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
