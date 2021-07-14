include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "Discovery")

set(CMAKE_SYSROOT /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/../..)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Makefile flags
set(CROSSDEV /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/../../toolchain/bin/arm-none-eabi-)
set(ARCH_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F407xx -D_TIMEVAL_DEFINED   -O0 -Wall -fdata-sections -ffunction-sections  ")
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
	/home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/lib/FreeRTOS-Plus-POSIX/include
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/lib/include
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/lib/include/private
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/lib/include/FreeRTOS_POSIX
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/lib/include/FreeRTOS_POSIX/sys
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/Inc
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/Drivers/STM32F4xx_HAL_Driver/Inc
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/Drivers/CMSIS/Device/ST/STM32F4xx/Include
 /home/mip-laptop/uros_frtos_ws/firmware/freertos_apps/microros_stm32f4_disco_extensions/Drivers/CMSIS/Include

    )
    
set(__BIG_ENDIAN__ 0)
