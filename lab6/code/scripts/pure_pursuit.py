#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# import ROS msg types and libraries
from nav_msgs.msg import Odometry
from pathlib import Path
import pandas as pd
import tf2_ros
import tf2_geometry_msgs
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math

# topic names
rviz_marker_topic = "waypoint_vis"
odom_topic = "odom"
drive_topic = 'nav'

# frame name
target_frame = "base_link"
global_frame = "map"

node_name = "pure_pursuit_node"
main_waypoint_filepath = Path(rospy.get_param("/f1tenth/waypoint_file"))
lookahead_dist = 0.5
steering_gain = 1 # gain k for the steering angle

main_waypoint_df = pd.read_csv(main_waypoint_filepath)

def get_transformed_point(row, trans):
    initial_point = PointStamped()
    initial_point.point.x = row["pos_x"]
    initial_point.point.y = row["pos_y"]
    transformed_point: PointStamped = tf2_geometry_msgs.do_transform_point(initial_point, trans)

    return transformed_point

def get_waypoint(trans):
    """Select a waypoint based on its x and y, in the car frame
    """
    main_waypoint_df["trans_pt"] = main_waypoint_df.apply(lambda row: get_transformed_point(row, trans), axis=1)
    main_waypoint_df["trans_x"] = main_waypoint_df["trans_pt"].apply(lambda row: row.point.x)
    main_waypoint_df["trans_y"] = main_waypoint_df["trans_pt"].apply(lambda row: row.point.y)
    main_waypoint_df["dist"] = (main_waypoint_df["trans_x"]**2 + main_waypoint_df["trans_y"]**2)**0.5

    # only choose points in front of the car, to prevent going backwards
    ahead_waypoint_df = main_waypoint_df[main_waypoint_df["trans_x"] > 0]

    # choose the waypoint with distance closest to lookahead distance
    idx = abs(ahead_waypoint_df["dist"]-lookahead_dist).argmin()
    point = ahead_waypoint_df.iloc[idx]

    return point["pos_x"], point["pos_y"], point["trans_x"], point["trans_y"], point["dist"]

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # create ROS subscribers and publishers.
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.pose_callback)
        self.current_waypoint_pub = rospy.Publisher(rviz_marker_topic, Marker, queue_size=10)

        # "listen" for the transformation
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def visualize_waypoint(self, x, y):
        marker = Marker()
        marker.header.frame_id = global_frame
        marker.header.stamp = self.time_stamp
        marker.id = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        return marker
    def pose_callback(self, pose_msg):
        try:
            # get transformation from global frame to local frame
            trans = self.tfBuffer.lookup_transform(target_frame, global_frame, rospy.Time())
        except:
            print("cannot find transform")
            return

        # find the current waypoint to track using methods mentioned in lecture
        x, y, trans_x, trans_y, L = get_waypoint(trans)

        # calculate curvature/steering angle
        angle = steering_gain * 2*trans_y/(L**2)
        if angle > 0.4189:
            angle = 0.4189
        elif angle < -0.4189:
            angle = -0.4189

        # change velocity depending on the steering angle
        velocity = 1.0
        self.time_stamp = rospy.Time.now()
        if 0 <= abs(angle) <= 10/180*math.pi :
            velocity = 1.5
        elif abs(angle) <= 20/180*math.pi:
            velocity = 1.0
        else: 
            velocity = 0.5

        # publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.time_stamp
        drive_msg.header.frame_id = target_frame
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        marker = self.visualize_waypoint(x, y)
        self.current_waypoint_pub.publish(marker)


def main():
    rospy.init_node(node_name)
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()