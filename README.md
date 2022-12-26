# Cartbot 

## 1. Overview

This package is for based on kalman filter and particle filter. 

<img width="100%" src="https://user-images.githubusercontent.com/46801826/209517737-1f17f7af-8902-47cf-b67f-ced679e96b8a.gif"/>

------



# 2. Prerequisites

You should preinstall Ubuntu 20.04 and Ros Melodic to Run

### 2.1 Eigen3

```
sudo apt-get install libeigen3-dev
```

### 2.2 PCL

```
sudo apt-get install ros-noetic-pcl-ros
```

### 2.3 graph_rviz_plugin

*graph_rviz_plugin* is used for plotting kalman filter ouptut performance. You can install in https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin

### 2.4 YDLIDAR ROS Driver

*YDLIDAR ROS Driver* is used for getting Point Cloud message. Because, We used YDLIDAR G4 lidar for this System. You can install in https://github.com/YDLIDAR/ydlidar_ros_driver 

------



# 3. System Architecture

<img width="100%" src="https://user-images.githubusercontent.com/46801826/209523105-5b1b4aea-8a48-4810-a53b-8cc20f722272.png"/>

#### 3.1 Node Overview

| Node Name       | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| Lidar Node      | YD LIDAR Ros Driver to get Point cloud message               |
| Clustering Node | ABD(Adaptive Breakpoint Detector) & DBSCAN => Adaptive Epsilon DBSCAN Based Clustering Node |
| Tracking Node   | Particle FIlter & Kalman Filter(Choose 1) based Tracking Node |
| Avoiding Node   | Accompany System(Driving algorithm) &  Potential Field based Avoiding Node |

#### 3.2 Message Overview

| Message Name | composition                                                  | Description                                |
| ------------ | ------------------------------------------------------------ | ------------------------------------------ |
| Cluster      | 1. distance <br />2. center coordinate(x,y)<br />3. geometry_msgs/point | Clustered data by Adaptive Epsiolon DBSCAN |
| ClusterArray | 1. Cluster[]                                                 | Vector array of Cluster                    |
| Current      | 1. center coordinate(x,y)<br />2. state of Tracking Node     | Tracking data processed by Kalman filter   |
| Encoder      | 1. left encoder count<br />2. right encoder count            | Encoder count data of each motor           |
| Speed        | 1. left motor speed(RPM)<br />2. right motor speed(RPM)      | Target RPM of each motor                   |

#### 3.3 ROS Parameter Overview

You can modify parameter file which is located in config/param.yaml

| Parameter name  | Description                                                  | Type         |
| --------------- | ------------------------------------------------------------ | ------------ |
| is_simulation   | true : Gazeo Simultion Mode<br />false: Real World Mode      | boolean      |
| min_pts         | minimum points to make object in DBSCAN                      | double       |
| min_eps         | minimum range to including point in same object              | double       |
| window_boundary | Minimum area for clustering                                  | double       |
| dist_threshold  | Reference distance to detect starting motion                 | double       |
| min_range       | Minimum distance to select Kalman filter measurement input   | double       |
| wait_time       | Time for Initializing target coordinate                      | double       |
| stop_time       | Time until the mode changes from STOP to LOST                | double       |
| A               | State Transtition Matrix for Kalman Filter                   | double array |
| C               | Observation Matrix for Kalman Filter                         | double array |
| R               | Measurement Noise Covariance Matrix for Kalman Filter        | double array |
| P               | State Covariance Matrix for Kalman Filter                    | double array |
| sigma_a         | Process Noise Covariance Matrix for Kalman Filter            | double       |
| Kp_rel          | Potential Field repulsive force Gain                         | double       |
| Kp_att          | Potential Field attractive force Gain                        | double       |
| min_dist        | Minimum distance to detect obstacles                         | double       |
| u_k             | Friction constant for Potential Field                        | double       |
| avoid_time      | Time to maintain the changed coordinates after an obstacle appears | double       |
| target_r        | Target Coordinates that change when an obstacle appears      | double       |
| isadvanced      | true : Advanced Accompany System<br />false: Accompany System | boolean      |
| wheel_radius    | radius of wheel                                              | double       |
| wheel_interval  | distance between wheels                                      | double       |
| base_range      | Distance between center of lidar and wheel axle              | double       |
| weight          | weight of robot (default : 15kg )                            | double       |
| max_err_x       | x component anti wind up limit for Accompany System          | double       |
| max_err_y       | y component anti wind up limit for Accompany System          | double       |
| kp_x            | x component velocity P gain for Accompany System             | double       |
| kp_y            | y component velocity P gain for Accompany System             | double       |
| ki_x            | x component velocity I gain for Accompany System             | double       |
| ki_y            | y component velocity I gain for Accompany System             | double       |

------

# 6. BackGround

### 6.1 Adaptive Epsilon DBSCAN

### 6.2 Kalman Filter

### 6.3 Potential Field 

------



# 5. How to Run

### With gazebo simulation

1. Change param.yaml

   ```yaml
   is_simulation: true
   ```

2. Run cartbot Simulation

   ```
   roslaunch cartbot simul.launch
   ```

### In real world

1. Change param.yaml

   ```yaml
   is_simulation: false
   ```

2. Give USB Permisson to LIDAR & Serial Port.(※ If you already enroll symbolic link for USB port, you can use name of custom port name that you already enrolled)

   ```
   sudo chmod +x /dev/ttyUSB0 #-- Lidar port
   sudo chmod +x /dev/ttyUSB1 #-- Serial port
   ```

2. Run YD LIDAR ROS Driver 

   ```
   roslaunch ydlidar_ros_driver lidar.launch
   ```

3. Run cartbot 

   ```
   roslaunch cartbot run.launch
   ```
