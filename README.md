# Hybrid-Force-Impedance-Control

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

`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=169.254.120.1 calibration_file:="${HOME}/my_robot_calibration.yaml"`

2. Launch the MoveIt:

`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e`


## Reference

1. [Compliant Control and Application](https://github.com/MingshanHe/Compliant-Control-and-Application)
2. [ROS2 Control Gazebo Tutorial Robot Simulation (Custom Robot Arm TeslaBot!)](https://www.youtube.com/watch?v=PM_1Nb9u-N0&t=618s)
3. [ROS 2 from Scratch: Get started with ROS 2 and create robotics applications with Python and C++](https://www.amazon.com/ROS-Scratch-started-robotics-applications-ebook/dp/B0DHV4VF5C/ref=pd_ci_mcx_mh_mcx_views_0_title?pd_rd_w=IiDWJ&content-id=amzn1.sym.bb21fc54-1dd8-448e-92bb-2ddce187f4ac%3Aamzn1.symc.40e6a10e-cbc4-4fa5-81e3-4435ff64d03b&pf_rd_p=bb21fc54-1dd8-448e-92bb-2ddce187f4ac&pf_rd_r=K5G7GD7MP6PD66N2PMF6&pd_rd_wg=1hXqi&pd_rd_r=f1eb68f0-bdba-4650-9ea4-e55823d3c8d2&pd_rd_i=B0DHV4VF5C)
