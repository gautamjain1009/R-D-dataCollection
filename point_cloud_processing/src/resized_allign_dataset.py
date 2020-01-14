#!/usr/bin/env python

import numpy as np 
import cv2
import os 
import natsort 


def resize(size,path_save_image,path_directory):
    index = 0 
    list_files = os.listdir(path_directory)
    list_files = natsort.natsorted(list_files,reverse =False)
    for path in list_files:
        full_path = os.path.join(path_directory, path)
        print(full_path)
        if os.path.isfile(full_path):
            # print("hello")
            index = index + 1 
            image = cv2.imread(full_path)
            resized_image = cv2.resize(image,size)
            # print(path_save_image)
            cv2.imwrite( path_save_image + str(index) + ".png",resized_image)

def sparsify_depth(path_save_image,path_directory):
    index = 0 
    list_files = os.listdir(path_directory)
    list_files = natsort.natsorted(list_files,reverse =False)
    for path in list_files:
        full_path = os.path.join(path_directory, path)
        print(full_path)
        if os.path.isfile(full_path):
            # print("hello")
            index = index + 1 
            image = cv2.imread(full_path)
            
            # resized_rgb= np.array(image, dtype=np.uint16) # This line only change the type, not values
            # resized_rgb *= 255
        
            # print("After conversion",resized_rgb.dtype)
        
            # resized_rgb = resized_rgb[:,:,0]
            # print(resized_rgb.shape)
            #making the realsenes data sparse 
            """
            Every second and third pixel is giving a value zero and every third row of pixels are set to zero.
            uncomment the below lines when do not needed a sparse images.
            """
            # for i in range(resized_rgb.shape[0]):
            #     for j in range(resized_rgb.shape[1]):
            #         if j % 2 == 0: 
            #             resized_rgb[i][j] = 0
            #         elif j% 3 ==0:
            #             resized_rgb[i][j] = 0

            # image = np.array(image, dtype=np.uint16)
            cv2.imwrite( path_save_image + str(index) + ".png",image)


if __name__ =="__main__":
    
    # #resized RGB_image
    # size_RGBimage = (1216,352)
    # path_RGB = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/RGB"
    # save_path_RGB = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/evaluation_dataset/bigger_RGB/kitti_res_rgb_"
    # resize(size_RGBimage,save_path_RGB,path_RGB)

    # #resized depth_image
    # size_depthimage = (1216,352)
    # path_depth = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/Depth"
    # save_path_depth = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/evaluation_dataset/resized_depth/kitti_res_depth_"
    # resize(size_depthimage,save_path_depth,path_depth)   

     #sparsifying the depth image
    path_depth = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/evaluation_dataset/resized_alligned_RGB"
    save_path_depth = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/evaluation_dataset/samenmed_resized_alligned_RGB/depth_image_"
    sparsify_depth(save_path_depth,path_depth)