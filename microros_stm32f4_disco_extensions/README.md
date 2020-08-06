# STM32F407G-Discovery1 Micro-ROS  Extension
Test conditions :
  ROS2  : dashing
  Board : STM32F407G-Dis1
  Hardware link :  CP210x UART Bridge (through UART2)
  Apps : int32_publisher, ping_pong

Before running this, make sure micro-ROS is built and sourced correctly
### 1. Create
```
ros2 run micro_ros_setup create_firmware_ws.sh freertos stm32f4_disco
```
This will create the firmware folder, if all extensions are not present you can download from the concerning repo
### 2. Configure

```
ros2 run micro_ros_setup configure_firmware.sh <app_name> --transport serial --dev 2
```
### 3. Build
This will first build microros followed by the board firmware
```
ros2 run micro_ros_setup build_firmware.sh
```

### 4. Flash
```
ros2 run micro_ros_setup flash_firmware.sh
```
Flashing is done via st-link but it can be changed to other methodes in the config

### Prepare Micro-ROS Agent
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
where `[device]` is likely `/dev/ttyUSB0` or you can try `dmesg |grep tty`

You should see the following
```
[1596701204.291016] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1596701205.760299] info     | Root.cpp           | create_client            | create                 | client_key: 0xF005BA11, session_id: 0x81
[1596701205.760416] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x4026907153, address: 1
```
You can also `-v6` add the end of the line for debugging
### Monitor messages
In a new terminal
```
cd uros_ws
. install/setup.bash
ros2 topic list
```
As per the app you are running you will get the topics related to them
```
