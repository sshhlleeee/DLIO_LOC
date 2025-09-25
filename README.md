# DLIO_LOC
This repository contains the implementation enabling [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) to perform localization on a prior point cloud map. 
(🚧 Under construction, `/initialpose` set z as 0 🚧)

### Dependencies
The following has been verified to be compatible, although other configurations may work too:

- Ubuntu 22.04
- ROS Humble (`rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_ros`)
- C++ 14
- CMake >= `3.12.4`
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10.0`
- Eigen >= `3.3.7`

### Execution

`ros2 launch direct_lidar_inertial_odometry dlio_loc.launch.py`

## Reference
```bibtex
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}
```
