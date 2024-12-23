# GRIL-Calib [ROS1 / ROS2]
Official implementation of our paper **"GRIL-Calib: Targetless Ground Robot IMU-LiDAR Extrinsic Calibration Method using Ground Plane Motion Constraints"**.  

- ArXiv : [https://arxiv.org/abs/2312.14035](https://arxiv.org/abs/2312.14035)  
- IEEE : [https://ieeexplore.ieee.org/document/10506583](https://ieeexplore.ieee.org/document/10506583)  

## About GRIL-Calib
<p align="center"><img src="./figs/GRIL-Calib-overview.png" width = "500" ></p>  

- **GRIL-Calib** is the LiDAR-IMU calibration method for ground robots.
- Using only **planar motion**, the 6-DOF calibration parameter could be estimated.

## 🚀 ROS2 Support  

If you want to use ROS2 version, check out `humble` branch.  

## Prerequisites (ROS1 version)
- Ubuntu 18.04
- ROS Melodic
- PCL >= 1.8
- Eigen >= 3.3.4
- [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
- [ceres-solver-2.0.0](http://ceres-solver.org/installation.html#linux)

### Set up your environment easily with Docker!  🐳  

**Requires [Docker](https://www.docker.com/) and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed.**

**1. Enter the `/docker` folder and make a docker image.**
```
git clone https://github.com/Taeyoung96/GRIL-Calib.git
```
```
cd GRIL-Calib/docker
```
```
docker build -t gril-calib .
```

When you have finished it, use the command `docker images` and you can see the output below.
```
REPOSITORY                   TAG                   IMAGE ID         CREATED          SIZE
gril-calib                   latest                9f90339349a0     5 months ago     3.78GB
```

**2. Make docker container (same path as above)**

In `/docker`,  
```
sudo chmod -R 777 container_run.sh
```
```
./container_run.sh <container_name> <image_name:tag>
```
**:warning: You should change {container_name}, {docker image} to suit your environment.**  

```
./container_run.sh gril-calib-container gril-calib:latest 
```

If you have successfully created the docker container, the terminal output will be similar to the below.
```
================Gril-Calib Docker Env Ready================
root@taeyoung-cilab:/root/catkin_ws#
```

**3. Build and run GRIL-Calib**

Inside the docker container, build and run the package.  
```
catkin_make
```
```
source devel/setup.bash
```

## Run with a public dataset  

The launch files for [M2DGR](https://ieeexplore.ieee.org/abstract/document/9664374), [HILTI](https://arxiv.org/abs/2109.11316), and [S3E](https://arxiv.org/abs/2210.13723), as experimented with in the paper, are shown below.

- For M2DGR,
```
roslaunch gril_calib m2dgr_xxxx.launch
```

- For HILTI,
```
roslaunch gril_calib hilti_xxxx.launch
```
- For S3E,
```
roslaunch gril_calib s3e_xxxx.launch
```
- For my rosbag (c32w)

```
roslaunch gril_calib velodyne.launch
```

config 中 velodyne16.yaml 设置 topic

After running the launch file, you simply run the bag file for each sequence.  

## Run with your custom data

**:warning: This version only supports Spinning LiDAR (Velodyne, Ouster), not Solid-state LiDAR.**  

The reason for this is that the [LiDAR ground segmentation](https://github.com/url-kaist/patchwork-plusplus-ros) algorithm has only been tested on Spinning LiDAR.  
If we could achieve ground segmentation, we could theoretically do it for a Solid-state LiDAR like Livox Avia.   

- Make sure to acquire your data on an area with flat ground.
- It would be helpful to collect data as the ground robot draws an "8".
- Please make sure the unit of your input angular velocity is rad/s.

### Important parameters

Similar to [LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init), edit `config/xxx.yaml` to set the below parameters:  

- `lid_topic`: Topic name of LiDAR point cloud.
- `imu_topic`: Topic name of IMU measurements.
- `imu_sensor_height`: Height from ground to IMU sensor (meter)
- `data_accum_length`: A threshold to assess if the data is enough for calibration.
- `x_accumulate`: Parameter that determines how much the x-axis rotates (Assuming the x-axis is front)
- `y_accumulate`: Parameter that determines how much the y-axis rotates (Assuming the y-axis is left)
- `z_accumulate`: Parameter that determines how much the z-axis rotates (Assuming the z-axis is up)
- `gyro_factor`, `acc_factor`, `ground_factor`: Weight for each residual
- `set_boundary`: When performing nonlinear optimization, set the bound based on the initial value. (only translation vector)
- `bound_th`: Set the threshold for the bound. (meter) ⭐️  See the [ceres-solver documentation](http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres7Problem22SetParameterUpperBoundEPdid) for more information.

## Acknowledgments  

Thanks to [hku-mars/LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init) for sharing their awesome work!  
We also thanks to [url-kaist/patchwork-plusplus-ros](https://github.com/url-kaist/patchwork-plusplus-ros) for sharing LiDAR ground segmentation algorithm.  

# 安装问题

1.安装遇到报错：
```
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "livox_ros_driver"
  with any of the following names:

    livox_ros_driverConfig.cmake
    livox_ros_driver-config.cmake
```

**解决方法：**

安装livox_driver:

```    
cd ~/catkin_ws
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin_make
```
重新catkin_make前，加入

```
export CMAKE_PREFIX_PATH=/home/office2004/catkin_ws/livox_ros_driver/devel:$CMAKE_PREFIX_PATH
```

2.安装遇到报错：

pp.hpp:16:10: fatal error: jsk_recognition_msgs/PolygonArray.h: No such file or directory
   16 | #include <jsk_recognition_msgs/PolygonArray.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

sudo apt-get install ros-noetic-jsk-recognition-msgs

## 标定结果

![Screenshot from 2024-12-23 11-40-26](https://github.com/user-attachments/assets/bf25a258-e837-4d4d-b781-d28d864aeb4d)
```
[Data accumulation] Rotation around Lidar X Axis complete! 
[Data accumulation] Rotation around Lidar Y Axis complete! 
[Data accumulation] Rotation around Lidar Z Axis complete! 
[Rotation matrix Ground to LiDAR (euler)]   0.6598   0.4154 0.002392 deg
[Rotation matrix Ground to IMU (euler)] -2.865  5.758  158.1 deg
[Estimated LiDAR sensor height] : 2.075 m
[calibration] Data accumulation finished, Lidar IMU calibration begins.

============================================================ 

Max Cross-correlation: IMU lag wtr Lidar : 0
Time lag 1: IMU lag wtr Lidar : 0
============================================================ 

[Calibration Result] Rotation matrix from LiDAR frame to IMU frame    = -2.465799 -0.346237  3.863721 deg
[Calibration Result] Translation vector from LiDAR frame to IMU frame = -0.035136  0.345199  1.938451 m
[Calibration Result] Time Lag IMU to LiDAR    = 0.00012792 s 
[Calibration Result] Bias of Gyroscope        =  0.010000  0.010000 -0.005406 rad/s
[Calibration Result] Bias of Accelerometer    = -0.009217 -0.011067 -0.009621 m/s^2
============================================================ 

Gril-Calib : Ground Robot IMU-LiDAR calibration done.
[TIMER] Batch optimization time consuming  227.679296 ms
Failed to open /root/catkin_ws/src/result/traj.txt
save LiDAR trajectory !!
Rebuild thread terminated normally
[laserMapping-1] process has finished cleanly
log file: /home/office2004/.ros/log/be82b640-c0cf-11ef-a608-675e3a253327/laserMapping-1*.log
^[[A^C[rviz-2] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```
