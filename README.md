# Cartbot 

# 1. Overview

This package was created for Capsone Design in KwangWoon University. The subjcect of Capstone is a "Accompany robot using 2d lidar sensor".  Tracking algorithm is based on **Kalman filter** and **Adaptive Epsilon DBSCAN.** 
<p align="center">
   <img width="100%" src="https://user-images.githubusercontent.com/46801826/209517737-1f17f7af-8902-47cf-b67f-ced679e96b8a.gif"/>
</p>


# 2. Prerequisites

You should preinstall `Ubuntu 20.04` and `ROS Noetic` to Run

### 2.1 Eigen3

```
sudo apt-get install libeigen3-dev
```

### 2.2 PCL

```
sudo apt-get install ros-noetic-pcl-ros
```

### 2.3 jsk_rviz_plugins

```
sudo apt-get install ros-noetic-jsk-rviz-plugins
```

### 2.4 graph_rviz_plugin

`graph_rviz_plugin` is used for plotting kalman filter ouptut performance. You can install in this [link](https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin )

### 2.5 YDLIDAR ROS Driver

`YDLIDAR ROS Driver` is used for getting Point Cloud message. Because, We used YDLIDAR G4 lidar for this System. You can install in this [link](https://github.com/YDLIDAR/ydlidar_ros_driver) 



# 3. System Architecture

<p align="center">
   <img width="90%" src="https://user-images.githubusercontent.com/46801826/209523105-5b1b4aea-8a48-4810-a53b-8cc20f722272.png" align="center"/>
</p>

### 3.1 Node Overview

| Node Name         | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| `Lidar Node`      | YD LIDAR Ros Driver to get PointCloud message                |
| `Clustering Node` | ABD(Adaptive Breakpoint Detector) & DBSCAN => Adaptive Epsilon DBSCAN based clustering |
| `Tracking Node`   | Particle FIlter & Kalman Filter(Choose 1) based tracking     |
| `Avoiding Node`   | Accompany System(Driving algorithm) &  Potential Field based avoiding |

### 3.2 Message Overview

| Message Name   | composition                                                  | Description                                |
| -------------- | ------------------------------------------------------------ | ------------------------------------------ |
| `Cluster`      | 1. distance <br />2. center coordinates(x,y)<br />3. geometry_msgs/point | Clustered data by Adaptive Epsiolon DBSCAN |
| `ClusterArray` | 1. `Cluster`[]                                               | Vector array of Cluster                    |
| `Current`      | 1. center coordinates(x,y)<br />2. state of Tracking Node    | Tracking data processed by Kalman filter   |
| `Encoder`      | 1. left encoder count<br />2. right encoder count            | Encoder count data of each motor           |
| `Speed`        | 1. left motor speed(RPM)<br />2. right motor speed(RPM)      | Target RPM of each motor                   |

### 3.3 ROS Parameter Overview

​	You can modify **yaml file** which is located in `config/param.yaml`

| Parameter name    | Description                                                  | Type         |
| ----------------- | ------------------------------------------------------------ | ------------ |
| `is_simulation`   | true : Gazeo Simultion Mode<br />false: Real World Mode      | boolean      |
| `min_pts`         | minimum points to make object in DBSCAN                      | double       |
| `min_eps`         | minimum range to including point in same object              | double       |
| `window_boundary` | Minimum area for clustering                                  | double       |
| `dist_threshold`  | Reference distance to detect starting motion                 | double       |
| `min_range`       | Minimum distance to select Kalman filter measurement input   | double       |
| `wait_time`       | Time for Initializing target coordinate                      | double       |
| `stop_time`       | Time until the mode changes from STOP to LOST                | double       |
| `A`               | State Transtition Matrix for Kalman Filter                   | double array |
| `C`               | Observation Matrix for Kalman Filter                         | double array |
| `R`               | Measurement Noise Covariance Matrix for Kalman Filter        | double array |
| `P`               | State Covariance Matrix for Kalman Filter                    | double array |
| `sigma_a`         | Process Noise Covariance Matrix for Kalman Filter            | double       |
| `Kp_rel`          | Potential Field repulsive force Gain                         | double       |
| `Kp_att`          | Potential Field attractive force Gain                        | double       |
| `min_dist`        | Minimum distance to detect obstacles                         | double       |
| `u_k`             | Friction constant for Potential Field                        | double       |
| `avoid_time`      | Time to maintain the changed coordinates after an obstacle appears | double       |
| `target_r`        | Target Coordinates that change when an obstacle appears      | double       |
| `isadvanced`      | true : Advanced Accompany System<br />false: Accompany System | boolean      |
| `wheel_radius`    | radius of wheel                                              | double       |
| `wheel_interval`  | distance between wheels                                      | double       |
| `base_range`      | Distance between center of lidar and wheel axle              | double       |
| `weight`          | weight of robot (default : 15kg )                            | double       |
| `max_err_x`       | x component anti wind up limit for Accompany System          | double       |
| `max_err_y`       | y component anti wind up limit for Accompany System          | double       |
| `kp_x`            | x component velocity P gain for Accompany System             | double       |
| `kp_y`            | y component velocity P gain for Accompany System             | double       |
| `ki_x`            | x component velocity I gain for Accompany System             | double       |
| `ki_y`            | y component velocity I gain for Accompany System             | double       |

------



# 6. BackGround

### 6.1 Adaptive Epsilon DBSCAN

**Adaptive Epsilon DBSCAN** is an algorithm that combines a **DBSCAN** and **ABD**(Adaptive Breakpoint Detector). We use Adaptive Epsilon DBSCAN to clustering object.

* **ABD(Adaptive Breakpoint Detector)**

$$
eps = r_{n-1}\frac{sin\Delta \theta }{sin(\lambda -\Delta \theta )}+3\sigma _{r}
$$

<p align="center">
   <img width="40%" src="https://user-images.githubusercontent.com/46801826/209551604-070d6195-98ab-4ab7-addf-553d70725f60.png" align="center"/>
</p>

* **DBSCAN(Density-based spatial clustering of applications with noise**)

   *<u>You can change `min_pts`and `min_eps` parameter in yaml file</u>*

<p align="center">
   <img width="50%" src="https://user-images.githubusercontent.com/46801826/209553660-2969290e-8747-4268-b9a5-56ca6862f0ab.png" align="center"/>
</p>

### 6.2 Kalman Filter

This package use **Kalman Filter** to estimate target coordinates. 

<p align="center">
   <img width="70%" src="https://user-images.githubusercontent.com/46801826/209535272-c6e50995-4e58-4623-8935-20fc6e1076d2.png" align="center"/>
</p>

* **State Transition Matrix**(*Under the assumption of **constant velocity motion model***)

$$
A=\begin{bmatrix}
1 & 0 & dt & 0 \\
0 & 1 & 0 & dt \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}\begin{bmatrix}
x \\
y \\
v_{x} \\
v_{y} \\
\end{bmatrix}
$$

* **Obseravation Matrix**

$$
H=\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

* **Process Noise Covariance Matrix** *<u>(You can change `sigma_a` parameter in yaml file)</u>*

$$
Q=\begin{bmatrix}
\Delta t^{2}\sigma_{vx}^{2} & 0 & \Delta t\sigma_{vx}^{2} & 0 \\
0 & \Delta t^{2}\sigma_{vy}^{2} & 0 & \Delta t^{2}\sigma_{vy}^{2} \\
\Delta t\sigma_{vx}^{2} & 0 & \sigma _{vx}^{2} & 0 \\
0 & \Delta t\sigma_{vx}^{2} & 0 & \sigma _{vy}^{2} \\
\end{bmatrix}
$$

### 6.3 Accompany System

Accompany System is driving algorithm for this package and register as a patent. You can check it out at the link below. http://kportal.kipris.or.kr/kportal/search/total_search.do **(patent number is `1020200100552`)**

<p align="center">
   <img width="50%" src="https://user-images.githubusercontent.com/46801826/209556746-00e5a262-3bfd-408a-a10e-166725556598.png" align="center"/>
</p>

------



# 5. User's Guide

### 5.1 With gazebo simulation

1. Change `param.yaml`

   ```yaml
   is_simulation: true
   ```

2. Run `simul.launch`

   ```
   roslaunch cartbot simul.launch
   ```

### 5.2 In real world

1. Change `param.yaml`

   ```yaml
   is_simulation: false
   ```

2. Give USB Permisson to LIDAR & Serial Port.(※ If you already enroll symbolic link for USB port, you can use name of custom port name that you already enrolled)

   ```
   sudo chmod +x /dev/ttyUSB0 #-- Lidar port
   sudo chmod +x /dev/ttyUSB1 #-- Serial port
   ```

3. Run YD LIDAR ROS Driver 

   ```
   roslaunch ydlidar_ros_driver lidar.launch
   ```

4. Run cartbot `run.launch`

   ```
   roslaunch cartbot run.launch
   ```

5. Working

   The robot works as follows Image.
   <p align="center">
      <img width="70%" src="https://user-images.githubusercontent.com/46801826/209533936-a9adff43-f0db-4ab0-9acd-23304cc3cd76.png"/>
   </p>
   <u>***You can change corresponding parameter in*** `config/param.yaml`**.**</u>
   
   
   | **State**  | Action | Condition (to Next State)                                    | **Parameter**        |
   | ---------- | ------ | ------------------------------------------------------------ | -------------------- |
   | `LOST`     | 정지   | 손바닥으로 15cm 이하의 거리로 LIDAR 가렸을 때<br /> ➨ `COUNT` | `dist_threshold`변경 |
   | `COUNT`    | 정지   | 4초 후(4초 동안 사용자 동행 위치 선정)<br /> ➨ `INIT`        | `wait_time`변경      |
   | `INIT`     | 정지   | Target 좌표 설정 후  <br />➨ `TRACKING`                      |                      |
   | `TRACKING` | 동작   | 10번 이상 Tracking 실패할 때 <br />➨ `LOST`<br />손바닥으로 15cm 이하의 거리로 LIDAR 가렸을 때  <br />➨ `STOP` | `dist_threshold`변경 |
   | `STOP`     | 정지   | 10초 후 <br />➨ `LOST`                                       | `stop_time` 변경     |
   
   

