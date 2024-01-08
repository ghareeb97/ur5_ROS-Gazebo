#!/usr/bin/env python3


import rospy, numpy as np
import cv2, cv_bridge
from sensor_msgs.msg import Image
from ur5_dynamics.msg import Tracker
from ur5_dynamics.msg import ColorDetected  # Import the custom message

tracker = Tracker()

class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)

        # Initialize tracking flags and variables
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0

        # Initialize the OpenCV bridge to convert ROS Image messages to OpenCV images
        self.bridge = cv_bridge.CvBridge()

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)

        # Publisher to publish tracked object's coordinates and color detection
        self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)
        self.color_pub = rospy.Publisher('color_detected', ColorDetected, queue_size=1)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for detecting blue, green, and red objects
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for each color
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine masks to detect blue, green, and red objects
        combined_mask = cv2.bitwise_or(blue_mask, green_mask)
        combined_mask = cv2.bitwise_or(combined_mask, red_mask1)
        combined_mask = cv2.bitwise_or(combined_mask, red_mask2)

        # Initialize color_detected variable to None
        color_detected = None

        # Determine the detected object's color based on the masks
        if np.any(blue_mask):
            color_detected = "Blue"
        elif np.any(green_mask):
            color_detected = "Green"
        elif np.any(red_mask1) or np.any(red_mask2):
            color_detected = "Red"

        # Find contours of the detected object
        (cnts, _) = cv2.findContours(combined_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w, d = image.shape

        M = cv2.moments(combined_mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Check if the area of the contour is greater than a threshold to confirm object detection
            for i, c in enumerate(cnts):
                area = cv2.contourArea(c)
                if area > 7500:
                    self.track_flag = True
                    self.cx = cx
                    self.cy = cy
                    self.error_x = self.cx - w/2
                    self.error_y = self.cy - (h/2 + 195)
                    tracker.x = cx
                    tracker.y = cy
                    tracker.flag1 = self.track_flag
                    tracker.error_x = self.error_x
                    tracker.error_y = self.error_y
                    cv2.circle(image, (cx, cy), 10, (0, 0, 0), -1)
                    cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx - 5), int(cy + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.drawContours(image, cnts, -1, (255, 255, 255), 1)
                    break
                else:
                    self.track_flag = False
                    tracker.flag1 = self.track_flag

        # Publish tracked object's coordinates and color detection
        self.cxy_pub.publish(tracker)
        self.color_pub.publish(ColorDetected(color_detected))

        # Display the camera image with object detection overlay
        cv2.namedWindow("End Effector Camera", 1)
        cv2.imshow("End Effector Camera", image)
        cv2.waitKey(1)

# Create an instance of the ur5_vision class
follower = ur5_vision()

# Run the ROS main loop
rospy.spin()
