#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from pathlib import Path
import pandas as pd

node_name = "pure_pursuit_node"
rviz_markers_topic = "visualization_marker_array"
frame_id = "map"
main_waypoint_filepath = Path("~/catkin_ws/logs/summary-wp-2021-10-09-04-31-06.csv")

main_waypoint_df = pd.read_csv(main_waypoint_filepath)

# TODO: import ROS msg types and libraries

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        pass

    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture
        pass

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians


def main():
    rospy.init_node(node_name)
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()