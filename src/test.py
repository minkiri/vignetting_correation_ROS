#!/usr/bin/env python3

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 미리 계산된 파라미터
a, b, c = 1.75, 0.75, -0.7421875
scale_factor = 0.5  # 축소 비율


class VignettingCorrectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("vignetting_correction", anonymous=True)

        # Subscribe to the input topic
        self.subscriber = rospy.Subscriber(
            "/camera5/image_color", Image, self.image_callback, queue_size=1
        )

        # Publisher for the corrected images
        self.publisher = rospy.Publisher(
            "/camera5/image_corrected", Image, queue_size=1
        )

        # CV bridge for converting ROS images to OpenCV images
        self.bridge = CvBridge()

        # Placeholder for precomputed distance map
        self.distance_map = None

        rospy.loginfo("Vignetting Correction Node Initialized")

    def precompute_distance_map(self, rows, cols):
        """
        Precompute the normalized distance map for the given image dimensions.
        """
        rospy.loginfo("Precomputing distance map...")
        c_x, c_y = cols / 2, rows / 2
        y, x = np.indices((rows, cols))
        distance_map = np.sqrt((x - c_x) ** 2 + (y - c_y) ** 2)
        max_distance = np.max(distance_map)
        return distance_map / max_distance

    def vignetting_correction_with_parameters(self, frame):
        """
        Apply vignetting correction using precomputed distance map.
        """
        rows, cols = frame.shape[:2]

        # Compute or reuse the distance map
        if self.distance_map is None or self.distance_map.shape[:2] != (rows, cols):
            self.distance_map = self.precompute_distance_map(rows, cols)

        # Compute gain map
        r2 = self.distance_map ** 2
        r4 = r2 ** 2
        r6 = r2 ** 3
        gain_map = 1 + a * r2 + b * r4 + c * r6

        # Apply gain map to the image
        corrected = frame.astype(np.float32) * gain_map[..., np.newaxis]
        return np.clip(corrected, 0, 255).astype(np.uint8)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Reduce the resolution
            frame_small = cv2.resize(
                frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR
            )

            # Apply vignetting correction
            corrected_small = self.vignetting_correction_with_parameters(frame_small)

            # Restore the original resolution
            corrected_frame = cv2.resize(
                corrected_small, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR
            )

            # Convert corrected OpenCV image back to ROS Image
            corrected_msg = self.bridge.cv2_to_imgmsg(corrected_frame, "bgr8")
            corrected_msg.header = msg.header

            # Publish the corrected image
            self.publisher.publish(corrected_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


if __name__ == "__main__":
    try:
        node = VignettingCorrectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

