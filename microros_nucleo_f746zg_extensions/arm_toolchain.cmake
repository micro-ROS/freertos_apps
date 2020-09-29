include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "LwIP")

set(CMAKE_SYSROOT /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/../..)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# External transports
# set(EXTERNAL_TRANSPORT_HEADER "@EXTERNAL_TRANSPORT_HEADER@" CACHE PATH "" FORCE)
# set(EXTERNAL_TRANSPORT_SRC "@EXTERNAL_TRANSPORT@" CACHE PATH "" FORCE) 

# Makefile flags
set(CROSSDEV /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/../../toolchain/bin/arm-none-eabi-)
set(ARCH_CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F746xx -Og -Wall -fdata-sections -ffunction-sections")
set(ARCH_OPT_FLAGS "")

# Compiler tools
foreach(tool gcc ld ar)
	string(TOUPPER ${tool} TOOL)
    find_program(${TOOL} ${CROSSDEV}${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${CROSSDEV}${tool}")
	endif()
endforeach()

set(CMAKE_C_COMPILER ${CROSSDEV}gcc)
set(CMAKE_CXX_COMPILER ${CROSSDEV}g++)

set(CMAKE_C_FLAGS_INIT "-std=c11 ${ARCH_CPU_FLAGS} ${ARCH_OPT_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${ARCH_CPU_FLAGS} ${ARCH_OPT_FLAGS} " CACHE STRING "" FORCE)


include_directories(SYSTEM 
	/home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/FreeRTOS-Plus-POSIX/include
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/include
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/include/private
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/include/FreeRTOS_POSIX
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/include/FreeRTOS_POSIX/sys
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/src/hal/interface
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/src/modules/interface
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/src/utils/interface
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/src/config
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/src/drivers/interface
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Middlewares/Third_Party/LwIP/src/include/posix
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Middlewares/Third_Party/LwIP/src/include
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Inc
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Drivers/STM32F7xx_HAL_Driver/Inc
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Drivers/CMSIS/Device/ST/STM32F7xx/Include
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Drivers/CMSIS/Include
 /home/eden/Desktop/uros_ws_freertos/firmware/freertos_apps/microros_nucleo_f746zg_extensions/Middlewares/Third_Party/LwIP/system

    )
    
set(__BIG_ENDIAN__ 0)