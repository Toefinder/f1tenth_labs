#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        angle_topic = '/angle'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.angle_pub = rospy.Publisher(angle_topic, Float32, queue_size=10)

    def preprocess_lidar(self, scan_msg: LaserScan):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges_raw = np.array(scan_msg.ranges)

        half_window = 3
        mvg_window = 2 * half_window + 1 # window for moving average
        # append np.nan values so that the resulting moving average array has the same length
        ranges = np.append(np.append(np.array([np.nan]*half_window), ranges_raw), np.array([np.nan]*half_window))
        ranges = np.convolve(ranges, np.ones(mvg_window), 'valid') / (mvg_window)
        # import ipdb; ipdb.set_trace()
        ranges[np.isinf(ranges) |
               (ranges > scan_msg.range_max) |
               (ranges < scan_msg.range_min)] = np.nan

        angle_ranges = np.arange(len(ranges_raw))*scan_msg.angle_increment - math.pi

        proc_ranges = ranges[(angle_ranges >= -75/180*math.pi) & (angle_ranges <= 75/180*math.pi)]
        angle_ranges = angle_ranges[(angle_ranges >= -75/180*math.pi) & (angle_ranges <= 75/180*math.pi)]

        # import ipdb; ipdb.set_trace()
        return angle_ranges, proc_ranges

    def find_max_gap(self, free_space_ranges: np.array):
        """ Return the start index & end index of the max gap in free_space_ranges

        The max gap should not include nan 
        """
        start_idx = 0
 
        max_length = 0
        curr_length = 0
        curr_idx = 0
        threshold = 2
        for k in range(len(free_space_ranges)):
            if free_space_ranges[k] > threshold:
                curr_length +=1
    
                # New sequence, store beginning index
                if curr_length == 1:
                    curr_idx = k
            else:
                if curr_length > max_length:
                    max_length = curr_length
                    start_idx = curr_idx
                curr_length = 0
        
        if curr_length > max_length:
            max_length = curr_length
            start_idx = curr_idx

        return start_idx, start_idx + max_length - 1
    
    def find_best_point(self, start_i: int, end_i: int, ranges: np.array):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # import ipdb; ipdb.set_trace()
        # return np.argmax(ranges[start_i: (end_i+1)]) + start_i
        return int((start_i + end_i)/2)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        angle_ranges, proc_ranges = self.preprocess_lidar(data)

        #Find closest point to LiDAR
        min_idx = np.argmin(proc_ranges)

        #Eliminate all points inside 'bubble' (set them to zero)
        rb = 8 # radius of bubble around the nearest point
        free_space_ranges = np.array(proc_ranges, copy=True)
        free_space_ranges[max(min_idx-rb, 0): min(min_idx+rb, len(free_space_ranges)-1)+1] = 0

        #Find max length gap
        start_idx, end_idx = self.find_max_gap(free_space_ranges)

        #Find the best point in the gap
        best_pt = self.find_best_point(start_idx, end_idx, free_space_ranges)

        #Publish Drive message
        angle = angle_ranges[best_pt]
        angle_velocity = 0

        if 0 <= abs(angle) <= 10/180*math.pi:
            velocity = 1
        elif abs(angle) <= 20/180*math.pi:
            velocity = 0.5
        else:
            velocity = 0.2
            angle_velocity = 1 # rad/s

        self.angle_pub.publish(angle)
        

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.steering_angle_velocity = angle_velocity
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
