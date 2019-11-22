#!/usr/bin/env python

import cv2
import numpy as np
import sys
import glob
import os


def onMouse(event, x,y,flags,param):
    """
    Mouse callback gives the position of the mouse when mouse is pressed.
    """
    if event==cv2.EVENT_LBUTTONDOWN:
            pts1.append((x,y))

def allign_warp(h,image):
    """
    To allign depth and RGB image using warpPerspective with calculated Homography Matrix
    Args : 
        1. h : Homography matrix between the RGB Image from camera and the depth image from the lidar
        2. image : image to be wraped

    """
    im_out = cv2.warpPerspective(image,h,(image.shape[1],image.shape[0]))
    
    return im_out

def save_image(h,path_directory,path_save_image):
    """
    save the warped images
    Args : 
    1. h :  Homography matrix between the RGB Image from camera and the depth image from the lidar
    2. path_directory : the path of all the directory of the images to be warped
    3. path_save_image : the path where all the images need to saved. 
    """

    index = 0
    for path in os.listdir(path_directory):
        full_path = os.path.join(path_directory, path)
        if os.path.isfile(full_path):
            image = cv2.imread(full_path)
            warped_image = allign_warp(h,image)
            # print(warped_image.shape)
            index = index +1 
            cv2.imwrite( path_save_image + str(index) + ".png",warped_image)
    
if __name__ == '__main__':
    
    pts1 = []

    cv2.namedWindow("image")
    cv2.setMouseCallback("image", onMouse)

    #reading and showing the first image
    a = cv2.imread(sys.argv[1])
    print("1st img")
    cv2.imshow("image", a)
    print("showing")
    cv2.waitKey(0)


    cv2.namedWindow("depth_image")
    cv2.setMouseCallback("depth_image", onMouse)
    #reading and showing the second image
    b = cv2.imread(sys.argv[2])
    print("2nd im")
    cv2.imshow("depth_image", b)
    print("showing")
    cv2.waitKey(0)

    h,status = cv2.findHomography(np.asarray(pts1[:4]), np.asarray(pts1[4:]))
    
    # print("Homography matrix: ", h)

    d = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/RGB"
    save = "/home/gautam/Documents/MAS_DOCS/R-D_approaches/hypersen_dataset/allign_RGB/alligned_rgb"
    save_image(h,d,save)

    print("All the images are alligned and saved")
    print("Done")