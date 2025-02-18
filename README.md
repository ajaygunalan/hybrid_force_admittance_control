# hybrid-force-admittance-control

Based on:
- [paper](https://doi.org/10.1109/LRA.2023.3270036)
- [video](https://www.youtube.com/watch?v=rm8Irnc8v2M)

## Dependencies
| Item           | Value                                                                    |
|----------------|--------------------------------------------------------------------------|
| Ubuntu         | 24.04.1 LTS                                                              |
| Linux Kernel   | 6.8.0-52-lowlatency                                                      |
| ROS2           | Jazzy                                                                    |
| Robot          | UR5e                                                                     |
| UR5e Driver    | [Link](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)  |
| MoveIt2        | [Link](https://moveit.ai/install-moveit2/binary/)                        |
| Gazebo         | [Link](https://gazebosim.org/docs/latest/ros_installation/)              |


## To Move the robot using MoveIt

1. Launch the UR5e drivers:

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=169.254.120.1 calibration_file:="${HOME}/my_robot_calibration.yaml"
```

2. Launch the MoveIt:

```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
```

3. To visualise the URDF:

```
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

## Basics

```
source /opt/ros/jazzy/setup.bash
```

```
source install/setup.bash
```

## To Simulate a Robotic Arm in Gazebo & ROS2

```
bash ~/ros2_ws/src/hybrid_force_admittance_control/ur5e_bringup/scripts/ur5e_gazebo.sh
```

```
ros2 run ur5e_system_tests  ur5e_loop_controller
```
## Admittance Control
## Hybrid Force Admittance Control

## Reference

1. [Compliant Control and Application](https://github.com/MingshanHe/Compliant-Control-and-Application)
2. [Automtic Addison](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/)
