#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image as SesnorImage
from cv_bridge import CvBridge , CvBridgeError
import numpy as np


class realsense_data:

    def __init__(self):

        #initilizing the subscribers here 
        self.sub = rospy.Subscriber('/device_0/sensor_0/Depth_0/image/data', SesnorImage,self.general_callback)
        # self.sub = rospy.Subscriber('/device_0/sensor_1/Color_0/image/data', SesnorImage,self.general_callback)
        # rospy.subscriber('/device_0/sensor_0/Depth_0/image/data', SesnorImage)
        self.index = 0  
        self.bridge = CvBridge()
        self._image_raw = SesnorImage()
        # self._image_depth = SesnorImage()
        self.size = (1216,352)

    def general_callback(self,image_raw):
        
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
        self.index = self.index + 1 
        
        # rospy.loginfo('hey')
        self._image_raw = image_raw

        # rospy.loginfo(self._image_raw.type)
        

        # #### RGB Image
        # RGB_name = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/realsense_dataset/RGB/RGB_image_" + str(self.index) + ".png"
        # self.ros_image2_rgbimage(self._image_raw,RGB_name)
        
        
        #### depth_image
        depth_name = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/realsense_dataset/Depth_check/depth_image_" + str(self.index) + ".png"
        self.ros_image2_rgbimage(self._image_raw,depth_name)


    def ros_image2_rgbimage(self,ros_image,name): 

        try:
            print("hello")
            cv_image = self.bridge.imgmsg_to_cv2(ros_image,'bgr8')
        
        except CvBridgeError as e:
            print(e)
        
        
        resized_rgb = cv2.resize(cv_image,self.size)
        print(resized_rgb.shape)

        resized_rgb= np.array(resized_rgb, dtype=np.uint16) # This line only change the type, not values
        resized_rgb *= 255
        
        print("After conversion",resized_rgb.dtype)
        
        resized_rgb = resized_rgb[:,:,0]
        print(resized_rgb.shape)

        cv2.imwrite(name,resized_rgb)

if __name__ == '__main__':
    rospy.init_node('realsense')
    realsense_data = realsense_data()
    rospy.spin()





