# Eirabot CAN bus

This package is used for lanching nodes and interacting with them with ROS. This package contains the bus configuration file for the CAN bus and 2 different ros2 launch files.

## Simulating Eirabot canbus

The wheels on the eirabot can be simulated using fake cia402 slaves. These slaves mimic a cia40 hardware interfaces. The joints in the URDF will be controlled by this slave. The launch file below uses fake cia402 slaves, ros2 control and rviz to visaluse the robots expected movement

To launch the simulated eirabot

```bash
colcon build --packages-select eirabot_can_bus
source install/setup.bash 
ros2 launch eirabot_can_bus
```

## Running Eirabot canbus

To launch CANbus nodes with ros on the eirabot uses the commands below

## Controlling robot with Keyboard
To control the robot with key board you can you use teleop twist but make sure to remap cmd_vel to /diffbot_base_controller/cmd_vel_unstamped

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```
