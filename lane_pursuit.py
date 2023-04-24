#!/usr/bin/env python

import rospy
import numpy as np
import math

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)
        self.relative_x = 0
        self.relative_y = 0
        self.wheelbase_length = 0.325

        self.VELOCITY = rospy.get_param('VELOCITY', 0.6)
        self.lookahead = rospy.get_param('lookahead',2.0)

    def relative_cone_callback(self, msg):
        """
        Drive towards msg.x, msg.y using pure pursuit.
        """
        relative_x = msg.x_pose
        relative_y = msg.y_pos
        L = self.wheelbase_length
        eta = math.atan2(relative_y, relative_x)
        
        self.steering_angle = math.atan(2*L*math.sin(eta)/self.lookahead)
        
        self.drive(self.speed, self.steering_angle)

        self.error_publisher()

    def drive(self, speed = 0, steering_angle = 0):
        """
        Publishes AckermannDriveStamped msg with speed, steering_angle, and steering_angle_velocity
        """
        # create drive object
        ack_drive = AckermannDrive()
        ack_drive.speed = speed
        ack_drive.steering_angle = steering_angle

        #create AckermannDriveStamped object
        ack_stamp = AckermannDriveStamped()
        ack_stamp.drive = ack_drive
        self.drive_pub.publish(ack_stamp)

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = math.sqrt(self.relative_x**2 + self.relative_y**2)
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
