This repository contains the dataset collection scripts for R&D project for depth completion and the title of the project is "Fusing 3d Range Measures with high resolution RGB data for obstacle detection and avoidance" 

## Contents
1. [Dependency](#dependency)
0. [Data](#data)
0. [Commands](#commands)
0. [Envrionment Specification](#envrionment specification)
0. [Report](#report)


## Dependency
The code was tested in ROS-Kinetic on Ubuntu 16.04.

- Data collection scripts are in `point_cloud_processing` 
```
To install ROS you can reffer to [ROS](https://www.ros.org/)
```
- To run the data processing scripts you need the following dependencies in ROS. 
```
#CV Bridge to link the OpenCV to ROS
sudo apt install ros-kinetic-cv-bridge
```
- Other local dependencies are: 
```
#OpenCV
pip3 install opencv-python

#Pypcl
pip install PyPCL==0.1.9 
```

- The drivers for the sensors used 

```
# HD webcam (Logitech webcam carl zeiss tessar)
(https://github.com/gautamjain1009/R-D-dataCollection) under folder named `my_camera`. 
 
# Hypersen 3d solid state Lidar
(https://github.com/ropod-project/hps_camera)

# Intel Realsense d435 3d camera 
- can be found under (https://github.com/IntelRealSense/realsense-ros) follow the instructions written there to install the driver. 
```

## Data
- Download the [KITTI Depth](http://www.cvlibs.net/datasets/kitti/eval_depth.php?benchmark=depth_completion) Dataset from their website. Use the following scripts to extract corresponding RGB images from the raw dataset. 

- Pre-processed data collected with Intel Realsense. (for raw data contact the author)
(https://drive.google.com/drive/folders/1ocrQCeEgyUfY9WrD8xkUOHOy6bDucdoX?usp=sharing)  

- Pre-processed data collected with Hypersen 3d Solid State lidar and RGB camera. (for raw data contact the author) 
(https://drive.google.com/drive/folders/1jhHfoTIcKJP5HG3gOxz3KfdxaOirw4rq?usp=sharing)

## Commands
- To execute the data processing commands for collected data
```
#Download the bag files collected on Intel Realsense or Hypersen and camera setup.   
rosbag play 20191122_1.bag #replace the name of the bag file

# To run the rosnode for data processing for data collected on Hypersen and camera setup.
rosrun point_cloud_processing dataScript.py 

# To run the rosnode for data processing for data collected on Hypersen and camera setup.
rosrun point_cloud_processing data_script_realsense.py
```
- Data resizing and RGb and depth allign scripts are `R-D-dataCollection/point_cloud_processing/src/resized_allign_dataset.p`, `R-D-dataCollection/point_cloud_processing/src/allign_rgb_depth.py`

```
# To run the RGB and depth allign script 
python resized_allign_dataset.py 
```

## Envrionment Specifications
I can not create a comman script or an environemtn for all the approaches together as the dependencies are different and comman dependencies are in different versions.

- The specifications files can be found in (https://github.com/gautamjain1009/R-D-dataCollection) under `R-D-dataCollection/Environment_specifiaction_files`.

## Report

- [Report](https://github.com/gautamjain1009/R-D-dataCollection/tree/master/report) 



