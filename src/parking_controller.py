#!/usr/bin/env python

import rospy
import numpy as np
import math

from final_challenge.msg import LaneLocation, PurePursuitError
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", LaneLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            PurePursuitError, queue_size=10)
        self.data_logger = rospy.Publisher("/data", String, queue_size=10)

        self.LineFollower = rospy.get_param("is_line_follower", False)

        self.parking_distance = 0.15 if self.LineFollower else 0.5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.VELOCITY = rospy.get_param('VELOCITY', 0.75)
        self.orientation = False

    def relative_cone_callback(self, msg):
        """
        Park car facing object given x_pos and y_pos
        TODO: Find way to vary approach velocity using safety controller code
        TODO: Find appropriate velocity for backing up
        TODO: Test to make sure cone kept in field of vision. 
            Reduce Lfwb if cone not kept in lookahead distance
        TODO: Workout trig to find minimum distance needed to avoid more than
        one two point turn
        """
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        L = 0.325 # Wheel base
        eta = math.atan2(self.relative_y, self.relative_x)
        cone_dist = math.sqrt(self.relative_x**2 + self.relative_y**2)
        rospy.loginfo("Cone Dist: " + str(cone_dist) + " x: " + str(self.relative_x) + " y: " + str(self.relative_y))
        setpoint = self.parking_distance
        threshold = .3 # meters
        
        # look_ahead = 0.5
        # steer_angle = math.atan(2*L*math.sin(eta)/look_ahead)
        # steer_angle = math.atan( (L*math.sin(eta)) / ((look_ahead/2) + (look_ahead* math.cos(eta))))

        look_ahead = 1 * self.VELOCITY
        steer_angle = math.atan(2*L*math.sin(eta)/look_ahead)
        #steer_angle = math.atan((L*math.sin(eta)) / ((look_ahead/2) + (look_ahead*math.cos(eta))))

        if self.orientation == False:
            if cone_dist > setpoint + threshold: # Cone too far
                self.drive(self.VELOCITY, steer_angle)

            elif cone_dist < setpoint - threshold: # cone too close
                self.orientation = True
                self.drive(-self.VELOCITY,-steer_angle)

            else: # cone within range
                # rospy.loginfo("Cone within range ")
                if abs(eta) > .15:
                    self.orientation = True
                    self.drive(-self.VELOCITY, -steer_angle)
                else:
                    self.drive(0,0)

        else:
            if (cone_dist > (setpoint - threshold) or (eta < 0.05)):
                self.orientation = False

            else:
                self.drive(-self.VELOCITY, -steer_angle)

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
        error_msg = PurePursuitError()

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
