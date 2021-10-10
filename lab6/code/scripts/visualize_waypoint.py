#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from pathlib import Path
import pandas as pd

node_name = "visualize_waypoint_node"
rviz_markers_topic = "/waypoint_vis_array"
frame_id = "map"
main_waypoint_filepath = Path(rospy.get_param("/f1tenth/waypoint_file"))
main_waypoint_df = pd.read_csv(main_waypoint_filepath)
marker_array_msg = MarkerArray()
def make_markers(row):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.id = row.name
    marker.pose.position.x = row["pos_x"]
    marker.pose.position.y = row["pos_y"]
    marker.color.r = 1
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    return marker

marker_array_msg.markers = main_waypoint_df.apply(make_markers, axis=1).tolist()

def main():
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher(rviz_markers_topic, MarkerArray, queue_size=1)
    while not rospy.is_shutdown():
        pub.publish(marker_array_msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
