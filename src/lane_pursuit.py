#!/usr/bin/env python

import rospy
import numpy as np
import math

from final_challenge.msg import LaneLocation, PurePursuitError
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from collections import deque

class LanePursuit():
    """
    A controller for applying Pure Pursuit to lane following
    """
    def __init__(self):
        rospy.Subscriber("/relative_lane", LaneLocation,
            self.relative_lane_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/lane_error",
            PurePursuitError, queue_size=10)
        self.relative_x = 0
        self.relative_y = 0
        self.wheelbase_length = 0.325
        self.max_turn = 0

        self.VELOCITY = rospy.get_param('VELOCITY', 0.75)
        self.lookahead = rospy.get_param('lookahead',2.5)

        self.x_list = deque([0 for i in range(2)])
        self.y_list = deque([0 for i in range(2)])

    def relative_lane_callback(self, msg):
        """
        Drive towards msg.x, msg.y using pure pursuit.
        """
        relative_x = msg.x_pos
        #relative_y = msg.y_pos - 0.14 Car 56
        relative_y = msg.y_pos - 0.14

        # Compute Running Average
        self.x_list.popleft()
        self.y_list.popleft()
        self.x_list.append(relative_x)
        self.y_list.append(relative_y)
        relative_x = sum(self.x_list) / float(len(self.x_list))
        relative_y = sum(self.y_list) / float(len(self.y_list))

        L = self.wheelbase_length
        eta = math.atan2(relative_y, relative_x)
        
        self.steering_angle = math.atan(2*L*math.sin(eta)/self.lookahead)
        
        self.drive(self.VELOCITY, self.steering_angle)

        if self.error_pub.get_num_connections() >= 0:
            self.error_publisher()

    def drive(self, speed = 0, steering_angle = 0):
        """
        Publishes AckermannDriveStamped msg with speed, steering_angle, and steering_angle_velocity
        """
        # create drive object
        ack_drive = AckermannDrive()
        ack_drive.speed = speed
        ack_drive.steering_angle = steering_angle
        #self.max_turn = max(steering_angle, self.max_turn)
        #print(self.max_turn)

        #create AckermannDriveStamped object
        ack_stamp = AckermannDriveStamped()
        ack_stamp.drive = ack_drive
        self.drive_pub.publish(ack_stamp)

    def error_publisher(self):
        """
        Publish the error between the car and the lane. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = PurePursuitError()

        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.lookahead = self.relative_x
        error_msg.horizontal_error = self.horizontal_error
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('LanePursuit', anonymous=True)
        LanePursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
