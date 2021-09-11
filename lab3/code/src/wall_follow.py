#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from rospy.core import NullHandler
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

#PID CONTROL PARAMS
kp = 0.5#TODO
kd = 0.05#TODO
ki = 0.001#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8

CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

lookahead_distance = 0.3

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        distance_topic = '/distance'
        error_topic = '/error'

        self.distance_pub = rospy.Publisher(distance_topic, Float32, queue_size=10)
        self.error_pub = rospy.Publisher(error_topic, Float32, queue_size=10)

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
        self.prev_secs = None
        self.prev_nsecs = None
        self.secs = None
        self.prev_nsecs = None

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        ranges = data.ranges
        real_angle = angle - 90 # transform the frames for angle
        ind  = int((real_angle + 180) / 360 * len(ranges))
        
        return ranges[ind]

    def pid_control(self, error, dt):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        #TODO: Use kp, ki & kd to implement a PID controller for 
        integral += error * dt

        angle = -(kp * error + kd * (error - prev_error) / dt + ki * integral)
        if 0 <= abs(angle) <= 10/180*math.pi :
            velocity = 1.5
        elif abs(angle) <= 20/180*math.pi:
            velocity = 1.0
        else: 
            velocity = 0.5

        prev_error = error

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


    # def followRight(self, data, rightDist):
    #     #Follow right wall as per the algorithm 
    #     a = self.getRange(data, 0)
    #     b = self.getRange(data, 20)
    #     theta = 20 / 180 * math.pi
    #     alpha = math.atan((a * math.cos(theta) - b)/ a * math.sin(theta))
    #     distance = b * math.cos(alpha) + LOOKAHEAD_DISTANCE * math.sin(alpha)

    #     error = rightDist - distance
    #     self.distance_pub.publish(distance)
    #     return error

    def followLeft(self, data, leftDist):
        global lookahead_distance
        #Follow left wall as per the algorithm 
        a = self.getRange(data, 160)
        b = self.getRange(data, 180)
        theta = 20 / 180 * math.pi
        alpha = math.atan((a * math.cos(theta) - b)/ a * math.sin(theta))
        distance = b * math.cos(alpha) + lookahead_distance * math.sin(alpha)

        error = leftDist - distance
        self.distance_pub.publish(distance)
        self.error_pub.publish(error)
        return error

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT) #TODO: replace with error returned by followLeft
        self.secs = data.header.stamp.secs
        self.nsecs = data.header.stamp.nsecs
        
        if self.prev_nsecs is None or self.prev_secs is None:
            self.prev_secs = self.secs
            self.prev_nsecs = self.nsecs
            return
            
        self.dt = (self.secs - self.prev_secs) + (self.nsecs - self.prev_nsecs) * 1e-9
        #send error to pid_control
        self.pid_control(error, self.dt)

        self.prev_secs = self.secs
        self.prev_nsecs = self.nsecs

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
