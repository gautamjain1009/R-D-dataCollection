#!/usr/bin/env python
import rospy 
import numpy
from sensor_msgs import point_cloud2 as pc2


def listener():
    rospy.init_node("processing",anonymous=True)

    rospy.Subscriber("/hps_camera/depth/points",pc2,callback)
    rospy.spin()


def callback(ros_cloud):
    rospy.loginfo("data recieved")
    rospy.loginfo("=============>")
    

    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])

        print("x ================>")
        print(points_list[0])
    return points_list

if __name__ == 'main':
    listener()
