# esp32-camera configuration

* Step 1: create and configure

  The `create_firmware_ws.sh` is the same as esp32.                                                                                                                   
  After the `create_firmware` step you need to add the esp32-camera repository to the esp-idf components directory with the following commands. 
 ```
 pushd microros_ws/firmware/toolchain/esp-idf/components
 
 git clone https://github.com/espressif/esp32-camera.git
 
 popd
 ```
  The configuration step is also the same as esp32.
* Step 2: menuconfig 
```
 Serial flasher config > Flash size (set to 4M) 
 
 Component config > ESP32-specific > Support for external, SPI-connected RAM` (set to true)
 
 Component config > Driver configurations > RTCIO configuration (set to TRUE)
 
 Camera configuration > Camera pins. (BOARD_ESP32CAM_AITHINKER is default)
 ```
 Another way to enable the above is with the `sdkconfig.defaults` file by adding these lines:
 ```
 CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
 CONFIG_RTCIO_SUPPORT_RTC_GPIO_DESC=y
 CONFIG_ESP32_SPIRAM_SUPPORT=y
 **PICK ONE OPTION**
 CONFIG_BOARD_ESP32CAM_AITHINKER=y **OR**  CONFIG_BOARD_WROVER_KIT=y
 ```
* Step 3 : build, flash and run a micro-ros-agent


* Step 4 : Run `view_image.py` 
