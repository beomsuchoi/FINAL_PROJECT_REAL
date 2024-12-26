# FINAL_PROJECT_INTERN


I published the screen of usb_camera using the following repository. https://github.com/ros-drivers/usb_cam.git

깃 클론
```cpp
cd {your workspace}
git clone https://github.com/beomsuchoi/FINAL_PROJECT_REAL.git
```

HOW TO USE IMU
1. 사이트 접속

```cpp
(https://www.e2box.co.kr/entry/EBIMU-9DOF-ROS2-Package) and download it at {your workspace}.
```

4. 빌드, 소싱

```cpp
cd ~/ros2_ws
colcon build
source install/setup.bash

```

4. 실행

```cpp

publisher : 
ros2 run ebimu_pkg ebimu_publisher

In my node :
ros2 run imu_final imu_node
```

How to use Vision
1. 깃 클론

```cpp
cd {your workspace}
git clone https://github.com/beomsuchoi/FINAL_PROJECT_REAL.git
```

2. 빌드, 소싱

```cpp
cd {your workspace}
colcon build --packages-select finalproject
source install/setup.bash

```
3. run

```cpp
ros2 run finalproject vision_node

```
