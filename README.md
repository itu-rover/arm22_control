# ROS Control package for rover 22' manipulator

## Installation
---
**NOTE**

This package has build dependencies on [ros_control_boilerplate (melodic-devel)](https://github.com/PickNikRobotics/ros_control_boilerplate), [serial](https://github.com/wjwwood/serial) and [rover_utils](https://github.com/burkap/rover_utils) pkg

```
  git clone -b melodic-devel https://github.com/PickNikRobotics/ros_control_boilerplate.git
  git clone https://github.com/wjwwood/serial
  git clone https://github.com/burkap/rover_utils
```

---

## Usage with simulation
Start the hardware interface and ROS controllers:

```roslaunch arm22_control control_sim.launch``` 

Start the serial echo server:

```rosrun arm22_control serial_echo.py```

You can see the results in RViz.

## Usage with real hardware

* First, check the serial connection.
* Then, return the arm to its 'home' position and reset the sensor positions. 

After that, run the following command to start the hardware interface and ROS controllers:

```roslaunch arm22_control control_hw.launch``` 

You can see the results in RViz.
