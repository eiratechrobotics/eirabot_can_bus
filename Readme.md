# Eirabot CAN bus

This package is used for lanching nodes and interacting with them with ROS. This package contains the bus configuration file for the CAN bus and 2 different ros2 launch files.

## Simulating Eirabot canbus

The wheels on the eirabot can be simulated using fake cia402 slaves. These slaves mimic a cia40 hardware interfaces. The joints in the URDF will be controlled by this slave. The launch file below uses fake cia402 slaves, ros2 control and rviz to visaluse the robots expected movement

To launch the simulated eirabot

```bash
colcon build --packages-select eirabot_can_bus
source install/setup.bash 
ros2 launch eirabot_can_bus eirabot_can_sim.launch.py 
```

## Running Eirabot canbus

To connect to the eirabot's psu, drive motors, turntable and cameras the eirabot_can_bus package can be used.

>:warning: Currently the turntable and PSU have some bugs that cause it to only work some of the times

To launch CAN nodes services use the following commands

```bash
# In terminal 1 run 
source install/setup.bash 
ros2 launch eirabot_can_bus eirabot_can.launch.py 
# In terminal 2 after the launch file has launched the nodes run
# Set up turntable motor
ros2 service call /turntable_joint/velocity_mode std_srvs/srv/Trigger
ros2 service call /turntable_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3380, subindex: 0, data: 18000}"
ros2 service call /turntable_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3321, subindex: 0, data: 800}"
ros2 service call /turntable_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3323, subindex: 0, data: 800}"

# Set up left wheel
ros2 service call /left_wheel_joint/velocity_mode std_srvs/srv/Trigger
ros2 service call /left_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3380, subindex: 0, data: 18000}"
ros2 service call /left_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3321, subindex: 0, data: 800}"
ros2 service call /left_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3323, subindex: 0, data: 800}"

# Set up right wheel
ros2 service call /right_wheel_joint/velocity_mode std_srvs/srv/Trigger
ros2 service call /right_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3380, subindex: 0, data: 18000}"
ros2 service call /right_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3321, subindex: 0, data: 800}"
ros2 service call /right_wheel_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x3323, subindex: 0, data: 800}"
```

#### Driving drive motors with teleop twist

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

#### Lifting the turntable

```bash
# Lift turntable up
ros2 service call /vertical_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x2300, subindex: 0, data: 1}"

# Drop turntable down
ros2 service call /vertical_joint/sdo_write canopen_interfaces/srv/COWrite "{index: 0x2300, subindex: 0, data: 0}"
```

## Controlling robot with Keyboard

To control the robot with key board you can you use teleop twist but make sure to remap cmd_vel to /diffbot_base_controller/cmd_vel_unstamped

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

### High Level Diagram

