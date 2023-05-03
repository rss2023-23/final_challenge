#!/usr/bin/env python

import numpy as np
import math
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from final_challenge.msg import LaneLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class LaneDetector():
    """
    A class for applying your lane detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_lane_px (LaneLocationPixel) : the coordinates of the lane in the image frame (units are pixels).
    """
    def __init__(self):
        # Subscribe to ZED camera RGB frames
        self.lane_pub = rospy.Publisher("/relative_lane_px", LaneLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/lane_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/right/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_lane_px topic; the homography transformer will
        # convert it to the car frame.

        # Convert ROS image to OpenCV Image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        lane_pursuit_point = cd_color_segmentation(image)
        pixel = LaneLocationPixel()
        pixel.u = lane_pursuit_point[0]
        pixel.v = lane_pursuit_point[1]
        self.lane_pub.publish(pixel)

        # Publish debug if needed
        if self.debug_pub.get_num_connections() >= 0:
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
