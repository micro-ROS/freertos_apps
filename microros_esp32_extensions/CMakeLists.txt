cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED IDF_PATH)
    message(FATAL_ERROR "The CMake varible IDF_PATH was not set!")
endif()

if(NOT DEFINED UROS_APP)
    message(FATAL_ERROR "The CMake varible UROS_APP was not set!")
endif()

include(${IDF_PATH}/tools/cmake/project.cmake)
project(${UROS_APP})
