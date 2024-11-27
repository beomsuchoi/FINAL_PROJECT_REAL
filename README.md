# FINAL_PROJECT_INTERN


I published the screen of usb_camera using the following repository. https://github.com/ros-drivers/usb_cam.git

HOW TO USE IMU
1. 사이트 접속

```cpp
(https://www.e2box.co.kr/entry/EBIMU-9DOF-ROS2-Package) and download it at {your workspace}.
```

2. 빌드, 소싱

```cpp
cd ~/ros2_ws
colcon build
source install/setup.bash

```

3. 실행

```cpp

publisher : 
ros2 run ebimu_pkg ebimu_publisher

subscriber :
ros2 run ebimu_pkg ebimu_subscriber

In my node :
ros2 run imu_final imu_node
```

