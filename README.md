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


# To Do
1. Arm loop controller is not working — needs to be fixed.  
2. implement admittance control.  
3. Hybrid force-motion controller.

## Basics

Do this in every terminal
```
source /opt/ros/jazzy/setup.bash
```

```
source install/setup.bash
```

Inside the `src` of your ROS2 workspace

```
git clone git@github.com:ajaygunalan/hybrid_force_admittance_control.git && cd ..
```

check the dependencies by:

```
rosdep install -i --from-path src --rosdistro jazzy -y
```

If successful, then do: 

```
colcon build
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
## Hybrid Force Motion Control




## Reference

1. [Compliant Control and Application](https://github.com/MingshanHe/Compliant-Control-and-Application)
2. [Automtic Addison](https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/)
