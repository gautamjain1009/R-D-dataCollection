#!/usr/bin/env python

import rospy
import cv2
import pcl
from sensor_msgs.msg import Image as SesnorImage
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import message_filters
from cv_bridge import CvBridge , CvBridgeError
import time 


class synchronizer:

    def __init__(self):
        """
        Initializing all the subscribers that need to be synced and defining the message type to be used. 

        ApproximateTimeSynchroniz - Syncing subscriber topics by the delay of 0.1 seconds 
        ts.registerCallback - is called every time when topics are synced.  

        """

        # self.pub_Image = rospy.Publisher('image_raw_sync', SesnorImage, queue_size=1)
        # self.pub_Cam_Info = rospy.Publisher('camera_info_sync', CameraInfo, queue_size=1)
        # self.pub_Lidar = rospy.Publisher('rslidar_points_sync', PointCloud2, queue_size=1)

        self.imageInput = message_filters.Subscriber('/image_raw', SesnorImage)
        self.depthInput = message_filters.Subscriber('/hps_camera/depth/image', SesnorImage)

        # self.cameraInfo = message_filters.Subscriber('/camera_info', CameraInfo)
        # self.lidar = message_filters.Subscriber('/hps_camera/depth/points', PointCloud2)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.imageInput
                                                    , self.depthInput
                                                    ], 1,0.1)
        self.ts.registerCallback(self.general_callback)
        self.bridge = CvBridge()
        self._image_raw = SesnorImage()
        # self._camera_info = CameraInfo()
        # self._rslidar_points = PointCloud2()
        self._image_depth = SesnorImage()
        self.index= 0
        
        ###### size of the both of the images
        self.size = (160,60)


    def general_callback(self, image_raw, image_depth):
        
        """
        Callback function for the synchronized topics where the synchronized image is passed. 
        Args: 

            1. image_raw : Synced image ros message
            2. image_depth : Synced depth image ros message 
        """        
        # print("hey")
        self.index = self.index + 1 
        self._image_raw = image_raw
        # print(self._image_raw.header.stamp)
        self._image_depth = image_depth
        # self._camera_info = camera_info
        # self._rslidar_points = rslidar_points

        # depth_image_stamp = self._image_depth.header.stamp + ".png"

        #### RGB Image
        RGB_name = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/realsense_dataset/RGB/RGB_image_" + str(self.index) + ".png"
        self.ros_image2_rgbimage(self._image_raw,RGB_name)
        
        
        #### depth_image
        depth_name = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/realsense_dataset/Depth/depth_image_" + str(self.index) + ".png"
        self.ros_image2_rgbimage(self._image_depth,depth_name)
        
        # self.ros_pcl_cloud(rslidar_points)
    
    def ros_image2_rgbimage(self,ros_image,name): 
        """
        Conversion of ROS image messages into openCV images to be saved on disk

        Args: 
            1. ros_image : ros image message 
            2. name : path and name of the image to be saved on disk
    
        """
        try:
            # print("hello")
            cv_image = self.bridge.imgmsg_to_cv2(ros_image,'bgr8')
        
        except CvBridgeError as e:
            print(e)

        
        if cv_image.shape[0] == 480:
            resized_rgb = cv2.resize(cv_image,self.size)
            print(resized_rgb.shape)
            cv2.imwrite(name,resized_rgb)
        else:
            cv2.imwrite(name,cv_image)
        

    """
    All of are  preprocessing step for point clouds 
    """
         
    def ros_pcl_cloud(self,ros_cloud): 
        """
        Converts ros point cloud message to pcl cloud format 

        Args : 
            1. ros_cloud : ros cloud message
        """
        points_list = []
        #converting ros pointcloud2 message into pcl cloud containing only XYZ points.
        for data in pc2.read_points(ros_cloud,skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

            # print(data)
        self.pcl_cloud = pcl.PointCloud()
        self.pcl_cloud.from_list(points_list)

        # print(self.pcl_cloud)
        # print(self.pcl_cloud.size)
        self.pass_through_filter(self.pcl_cloud)
        # self.pcl_cloud2_depth_image(self.pcl_cloud)

    def pass_through_filter(self,cloud):
        """
        Passthrough filter to filter out the points that are out of the range of the lidar by setting the limits 
        
        Args: 
            cloud : pcl cloud that needs to be filtered.
        """

        print('Cloud before filtering: ')
        # for i in range(0, cloud.size):
        #     print('x: ' + str(cloud[i][0]) + ', y : ' + str(`
        #         cloud[i][1]) + ', z : ' + str(cloud[i][2]))
        
        # print(cloud.width)
        # print(cloud.height)
        
        print(" Filtering the cloud through pass through filter")
        #remove the outliers that is the range of z which are outliers. 
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(0, 5)

        filtered_cloud = passthrough.filter()

        
        # print(filtered_cloud.size)
        # for i in range(0,filtered_cloud.size):
        #     print('x: ' + str(filtered_cloud[i][0]) + ', y : ' + str(
        #         filtered_cloud[i][1]) + ', z : ' + str(filtered_cloud[i][2]))
        
        # self.pcl_cloud2_depth_image(filtered_cloud)
 
        
    def pcl_cloud2_depth_image(self,point_cloud):
        """
        This function converts the pcl point cloud to a depth image

        Args: 
            1. point_cloud : pcl cloud that will convert  
        
        """
        
        # point_cloud = pcl.PointCloud_PointWithViewpoint()
    
        noise_level = 0.0
        min_range = 0.0   
        border_size = 1
        angular_resolution_x = 0.5
        angular_resolution_y = 1
        coordinate_frame = pcl.CythonCoordinateFrame_Type.CAMERA_FRAME
        range_image = point_cloud.make_RangeImage()
        range_image.CreateFromPointCloud(point_cloud,
                                        angular_resolution_x, pcl.deg2rad(
                                            76.0), pcl.deg2rad(32.0),
                                        coordinate_frame, noise_level, min_range, border_size)
        
        range_image_widget = pcl.pcl_visualization.RangeImageVisualization()
        range_image_widget.ShowRangeImage(range_image)
        print(range_image)
        print(coordinate_frame)

        



if __name__ == '__main__':
    rospy.init_node('synchronizer')
    synchronizer = synchronizer()
    rospy.spin()


