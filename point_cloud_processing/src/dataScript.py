#!/usr/bin/env python

# import rospy
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# import rosbag
# import sensor_msgs.msg
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
# from sensor_msgs.msg import PointCloud2
# # from sensor_msgs.msg import PointCloud



# def SyncData(image, cloud):
#     camera_sync = image
#     hps_pointCloud = cloud

#     print("=====================>")
#     print("Publishing the synced data")
#     # while True:
#     rospy.Publisher("ray_image_sync",Image,queue_size = 1)
#     rospy.Publisher("hps_points_sync",PointCloud2,queue_size =1)


 
# if __name__ == "__main__":
    
#     rospy.init_node('dataCollection', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
    
#     camera_sync = Image()
#     hps_pointCloud = PointCloud2()

#     tss = ApproximateTimeSynchronizer([Subscriber("/image_raw", sensor_msgs.msg.Image),Subscriber("/hps_camera/depth/points", sensor_msgs.msg.PointCloud2)],10,0.1)
    
#     tss.registerCallback(SyncData)
    
#     # while not rospy.is_shutdown():
#     rospy.spin()


import rospy
from sensor_msgs.msg import Image as SesnorImage
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import message_filters


class synchronizer:
    def __init__(self):
        self.pub_Image = rospy.Publisher('image_raw_sync', SesnorImage, queue_size=1)
        self.pub_Cam_Info = rospy.Publisher('camera_info_sync', CameraInfo, queue_size=1)
        self.pub_Lidar = rospy.Publisher('rslidar_points_sync', PointCloud2, queue_size=1)

        self.imageInput = message_filters.Subscriber('/image_raw', SesnorImage)
        self.cameraInfo = message_filters.Subscriber('/camera_info', CameraInfo)
        self.lidar = message_filters.Subscriber('/hps_camera/depth/points', PointCloud2)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.imageInput
                                                    ,self.cameraInfo
                                                    , self.lidar
                                                    ], 1,0.1)
        self.ts.registerCallback(self.general_callback)

        self._image_raw = SesnorImage()
        self._camera_info = CameraInfo()
        self._rslidar_points = PointCloud2()

    def general_callback(self, image_raw, camera_info, rslidar_points):
        
        print("hey")
        self._image_raw = image_raw
        self._camera_info = camera_info
        self._rslidar_points = rslidar_points

    def publisher(self):
        while True:
            self.pub_Image.publish(self._image_raw)
            self.pub_Cam_Info.publish(self._camera_info)
            self.pub_Lidar.publish(self._rslidar_points)


if __name__ == '__main__':
    rospy.init_node('synchronizer')
    synchronizer = synchronizer()
    synchronizer.publisher()
    rospy.spin()
