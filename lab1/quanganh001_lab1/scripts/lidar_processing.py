#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan as LaserScanMsg
from anh_roslab.msg import scan_range

class PublisherSubscriber:
    def __init__(self, listen_topic, closest_topic, farthest_topic, range_topic):
        rospy.init_node('subandpub', anonymous=True)
        self.closest_pub = rospy.Publisher(closest_topic, Float64, queue_size=10)
        self.farthest_pub = rospy.Publisher(farthest_topic, Float64, queue_size=10)
        self.range_pub = rospy.Publisher(range_topic, scan_range, queue_size=10)
        self.sub = rospy.Subscriber(listen_topic, LaserScanMsg, self.callback)
        self.msg = scan_range()

    def callback(self, data):
        scan_data = np.array(data.ranges)
        valid_scan_data = scan_data[~ np.isinf(scan_data) & ~ np.isnan(scan_data)]
        self.closest_point = min(valid_scan_data)
        self.farthest_point = max(valid_scan_data)
        self.msg.min_val = self.closest_point
        self.msg.max_val = self.farthest_point
        self.msg.header = data.header
        self.closest_pub.publish(self.closest_point)
        self.farthest_pub.publish(self.farthest_point)
        self.range_pub.publish(self.msg)        

if __name__ == '__main__':
    PublisherSubscriber("/scan", "/closest_point", "/farthest_point", "/scan_range")
    rospy.spin()

