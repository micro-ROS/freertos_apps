# Nucleo F446ZE Micro-ROS Example Extension
Tested with: dashing

This is a quick run through of the setup steps, for more details visit the tutorial [page](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/).
### Prepare 
   ### TODO: replace branch and repo once merged with remote repo
```
source /opt/ros/dashing/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b dashing_nucleo_ports https://github.com/alsaibie/micro-ros-build.git src/micro-ros-build

sudo apt update && rosdep update
ropsdep install --from-path src --ignore-src -y

colcon build
source install/local_setup.bash
```

### 1. Create
This will create the firmware folder, grab the freertos_apps repo and download the arm compiler into toolchain
```
ros2 run micro_ros_setup create_firmware_ws.sh freertos nucleo_f446ze
```
### 2. Configure
This will copy over the uros_transport interface and set the APP and transport cmake flags
```
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial --dev 3
```
### 3. Build 
This will first build libmicroros, then the nucleo firmware
```
ros2 run micro_ros_setup build_firmware.sh 
```
#### To do a fast build (exclude libmicroros)
```
ros2 run micro_ros_setup build_firmware.sh -f
```

### 4. Flash
This will flash via st-link using openocd, to change to st-utilities, change USE_STFLASH to true in config of this port
```
ros2 run micro_ros_setup flash_firmware.sh
```


### Prepare Micro-ROS Agent
Clone the agent package and build it
```
ros2 run micro_ros_setup create_agent_ws.sh
colcon build
source install/local_setup.bash
```
### Run and Connect Device
Start the Micro-ROS Agent, remember to reset the microcontroller to establish connection with the agent
```
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```
where `[device]` is likely `/dev/ttyACMX`

You should see the following 
```
[1593502835.873965] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1593502838.863322] info     | Root.cpp           | create_client            | create                 | client_key: 0x5851F42D, session_id: 0x81
[1593502838.863412] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x1481765933, address: 1
[1593502838.916573] info     | SessionManager.hpp | establish_session        | session re-established | client_key: 0x1481765933, address: 1
```
### Monitor messages
In a new terminal
```
cd uros_ws
. install/setup.bash
ros2 topic list
```
You should see `/microROS/ping` and `microROS/pong` topics, then
```
ros2 topic echo /microROS/ping
```
and you should see the following
```
stamp:
  sec: 80
  nanosec: 584000000
frame_id: '1979932169_1085377743'
---
stamp:
  sec: 85
  nanosec: 545000000
frame_id: '1089957932_1085377743'
---
```