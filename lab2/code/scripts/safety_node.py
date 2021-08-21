#!/usr/bin/env python
import rospy
import numpy as np

# TODO: import ROS msg types and libraries
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # create ROS subscribers and publishers.
        self.brake_publisher = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=10)
        self.brake_bool_publisher = rospy.Publisher("/brake_bool", Bool, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.min_ttc_publisher = rospy.Publisher("/ttc", Float64, queue_size=10)

        # The minimum threshold for TTC, below which we will stop the car
        # the cars deceleration (8.26 m/s^2) and velocity for keyboard driving (1.8 m/s)
        self.ttc_limit = 0.35

        self.brake_msg = AckermannDriveStamped()
        self.brake_msg.drive.speed = 0

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        self.angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        self.ranges = np.array(scan_msg.ranges)
        self.range_rates = np.maximum(self.speed * np.cos(self.angles), 1e-10)
        # only calculate for valid ranges data
        self.ttcs = (self.ranges/self.range_rates)[(self.ranges >= scan_msg.angle_min) &
                                                    (self.ranges <= scan_msg.angle_max)] 
        self.min_ttc = np.min(self.ttcs)
        self.min_ttc_publisher.publish(self.min_ttc)
        # TODO: publish brake message and publish controller bool
        if self.min_ttc < self.ttc_limit:
            self.brake_bool_publisher.publish(True)
            self.brake_publisher.publish(self.brake_msg)
        else:
            self.brake_bool_publisher.publish(False)


def main():
    rospy.init_node('safety_node', anonymous=True)
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()