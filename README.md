# OakD Isaac ROS
An implementation of Isaac VIO and SLAM using the OakD stereo camera and imu

## Video Demo

Note: This project is still under development. Optimizations for camera calibration, and IMU integration are still
being developed. Provided here is a short video demo of how the project is progressing.

[![Video Demo](https://img.youtube.com/vi/9tzRvupRecQ/default.jpg)](https://youtu.be/9tzRvupRecQ)


## Docker
This implementation is intended to be run using the ROS docker system Nvidia provides [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common).

To build and run the docker images, the provided script can be used. This script adds OakD support to the existing docker system provided by Nvidia.

```sh
./run_docker.sh
```

## Launch Visual SLAM

Plug in the camera via USB3. Inside the docker container, launch the applications to read sensor data and process Elbrus SLAM

```sh
ros2 launch oakd_s2 oakd_s2.launch.py
```

To visualize output with Rviz, open a new termal in the container with the `run_docker.sh` script, the run
```sh
./rviz.sh
```
