# ROS-Driver
# Quick Start Guide
```
 Note*  
 -This ROS driver only supports firmware version 2.0 or 2.0+. 
 -You can check your firmware version from Roborun+ console tab by querying - "?fid". 
 -If firmware is not the latest one then please update it with the latest one available on Roboteq website 
  or contact "techsupport.roboteq@mail.nidec.com".
```
This repository contains the ROS driver for Roboteq controllers. The package requires ROS system to be installed properly to your system  and proper connection of Roboteq controller. For detailed controller setup instructions, please refer to our documentation [here](https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file).

First, clone this repository to catkin_ws/src 
```
git clone https://github.com/enco77/ROS-Driver.git
cd catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

The `Roboteq motor controller driver` is the package for the Roboteq ROS-driver. Make sure not to change package name as it will change the definition in the Cmake and Package files. Open new terminal and copy these steps -

```
cd catkin_ws/
source devel/setup.bash
roslaunch roboteq_motor_controller_driver driver.launch
roslaunch roboteq_motor_controller_driver diff_odom.launch
```

The roboteq driver is designed to be dynamic and users can publish the controller queries as per their requirements. The publishing queries is not limited to any value. By default total 5 queries are published by launching this driver. Users can change or add queries in configuration file. For that go to config/query.yaml

```
port : "/dev/ttyACM0" 
baud : 115200  
wheelbase: 1.0     # Distance between 2 wheels
radius: 0.105      # Radius of wheel
gear_ratio: 24.69  # Gear Ratio of Motorwheel
max_rpm: 2650.0    # Maximum speed of motor
frequency : 50     # frequency

query:
 encoder_count : "?C"  # Read encoder ticks
 encoder_speed : "?S"  # Read encoder speed
 feedback : "?F"       # Read speed/position/torque of motor depeds on closed loop parameter
 fault_flag : "?FF"    # Fault flag status of controller that can occur during operatiom
 status_flag: "?FM"    # Report the runtime status of each motor


```
For Odometry paramaters , you can change  base_frame , odom_frame, encoder_topic_name, command_srv (for restart odom status) and etc
```
base_frame: "base_link"
odom_frame: "odom"
encoder_topic_name: "/roboteq_motor_controller_driver/encoder_count"
command_srv: "/roboteq_motor_controller_driver/dualchannel_command_service"
gear_ratio: 24.69
radius: 0.105
wheelbase: 1.0
rate: 5              # Publish rate of odom topic
ppr: 1024            # Pulse per revolution of encoder
encoder_max: 65536   # Max count of encoder 
encoder_min: -65536  # Min count of encoder
```
