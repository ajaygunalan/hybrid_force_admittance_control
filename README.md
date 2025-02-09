# Hybrid-Force-Impedance-Control

## Dependencies
| Item           | Value                                                                    |
|----------------|--------------------------------------------------------------------------|
| Ubuntu         | 24.04.1 LTS                                                              |
| Linux Kernel   | 6.8.0-52-lowlatency                                                      |
| ROS2           | Jazzy                                                                    |
| Robot          | UR5e                                                                     |
| UR5e Driver    | [Link](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)  |
| MoveIt2        | [Link](https://moveit.ai/install-moveit2/binary/)                        |


## To Move the robot using MoveIt

1. Launch the UR5e drivers:

`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=169.254.120.1 calibration_file:="${HOME}/my_robot_calibration.yaml"`

2. Launch the MoveIt:

`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e`
