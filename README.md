# Cartbot 

## 1. Overview

This package is based on **Kalman filter** and **Adaptive Epsilon DBSCAN.** 

<img width="100%" src="https://user-images.githubusercontent.com/46801826/209517737-1f17f7af-8902-47cf-b67f-ced679e96b8a.gif"/>

------



# 2. Prerequisites

You should preinstall `Ubuntu 20.04` and `Ros Noetic` to Run

### 2.1 Eigen3

```
sudo apt-get install libeigen3-dev
```

### 2.2 PCL

```
sudo apt-get install ros-noetic-pcl-ros
```

### 2.3 graph_rviz_plugin

`graph_rviz_plugin` is used for plotting kalman filter ouptut performance. You can install in https://gitlab.com/InstitutMaupertuis/graph_rviz_plugin

### 2.4 YDLIDAR ROS Driver

`YDLIDAR ROS Driver` is used for getting Point Cloud message. Because, We used YDLIDAR G4 lidar for this System. You can install in https://github.com/YDLIDAR/ydlidar_ros_driver 

------



# 3. System Architecture

<img width="100%" src="https://user-images.githubusercontent.com/46801826/209523105-5b1b4aea-8a48-4810-a53b-8cc20f722272.png"/>

### 3.1 Node Overview

| Node Name       | Description                                                  |
| --------------- | ------------------------------------------------------------ |
| Lidar Node      | YD LIDAR Ros Driver to get Point cloud message               |
| Clustering Node | ABD(Adaptive Breakpoint Detector) & DBSCAN => Adaptive Epsilon DBSCAN Based Clustering Node |
| Tracking Node   | Particle FIlter & Kalman Filter(Choose 1) based Tracking Node |
| Avoiding Node   | Accompany System(Driving algorithm) &  Potential Field based Avoiding Node |

### 3.2 Message Overview

| Message Name | composition                                                  | Description                                |
| ------------ | ------------------------------------------------------------ | ------------------------------------------ |
| Cluster      | 1. distance <br />2. center coordinate(x,y)<br />3. geometry_msgs/point | Clustered data by Adaptive Epsiolon DBSCAN |
| ClusterArray | 1. Cluster[]                                                 | Vector array of Cluster                    |
| Current      | 1. center coordinate(x,y)<br />2. state of Tracking Node     | Tracking data processed by Kalman filter   |
| Encoder      | 1. left encoder count<br />2. right encoder count            | Encoder count data of each motor           |
| Speed        | 1. left motor speed(RPM)<br />2. right motor speed(RPM)      | Target RPM of each motor                   |

### 3.3 ROS Parameter Overview

​	You can modify **yaml file** which is located in `config/param.yaml`

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

**Adaptive Epsilon DBSCAN** is an algorithm that combines a **DBSCAN** and **ABD**(Adaptive Breakpoint Detector). We use Adaptive Epsilon DBSCAN to clustering object.

* **ABD(Adaptive Breakpoint Detector)**

<img width="30%" src="https://user-images.githubusercontent.com/46801826/209551604-070d6195-98ab-4ab7-addf-553d70725f60.png" />
$$
𝒆𝒑𝒔=𝒓_{𝒏−𝟏}\frac{𝒔𝒊𝒏𝜟𝜽}{𝐬𝐢𝐧⁡(𝝀−𝜟𝜽)}+𝟑𝝈_𝒓
$$

* **DBSCAN(Density-based spatial clustering of applications with noise**)

   *<u>You can change `min_pts`and `min_eps` parameter in yaml file</u>*

<img width="50%" src="https://user-images.githubusercontent.com/46801826/209553660-2969290e-8747-4268-b9a5-56ca6862f0ab.png" />

### 6.2 Kalman Filter

This package use **Kalman Filter** to estimate target coordinates. 

<img width="70%" src="https://user-images.githubusercontent.com/46801826/209535272-c6e50995-4e58-4623-8935-20fc6e1076d2.png"/>

* **State Transition Matrix**(*Under the assumption of **constant velocity motion model***)
  $$
  A =\begin{bmatrix}
  1& 0 & dt & 0 \\
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
  H = \begin{bmatrix}
  1 & 0 & 0 & 0 \\
  0 & 1 & 0 & 0 \\
  0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 1 \\
  \end{bmatrix}
  $$

* **Process Noise Covariance Matrix** *<u>(You can change `sigma_a` parameter in yaml file)</u>*

$$
Q=\begin{bmatrix}
\Delta t^{2}\sigma_{vx}^{2}  & 0 & \Delta t\sigma_{vx}^{2} & 0 \\
0 & \Delta t^{2}\sigma_{vy}^{2} & 0 & \Delta t^{2}\sigma_{vy}^{2} \\
\Delta t\sigma_{vx}^{2} & 0 & \sigma _{vx}^{2} & 0 \\
0 & \Delta t\sigma_{vx}^{2} & 0 & \sigma _{vy}^{2} \\
\end{bmatrix}
$$

------



# 5. How to Run

### 5.1 With gazebo simulation

1. Change param.yaml

   ```yaml
   is_simulation: true
   ```

2. Run cartbot Simulation

   ```
   roslaunch cartbot simul.launch
   ```

### 5.2 In real world

1. Change param.yaml

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

5. Robot is Working 

   The robot works as follows Image.

   <img width="70%" src="https://user-images.githubusercontent.com/46801826/209533936-a9adff43-f0db-4ab0-9acd-23304cc3cd76.png"/>
