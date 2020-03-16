#   Copyright 2016 sean93.park@gmail.com
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

function(BuildTestApp APPNAME APPFILES)
    add_executable(${APPNAME} ${APPFILES})
    target_include_directories(${APPNAME} SYSTEM PRIVATE ${FREERTOS_INCLUDE})
    target_include_directories(${APPNAME} PUBLIC ${FREERTOS_SOURCE})
    target_include_directories(${APPNAME} PUBLIC ${PATH_INCLUDE})
    target_include_directories(${APPNAME} PUBLIC ${PATH_PORTABLE})
    target_include_directories(${APPNAME} PUBLIC ${CMAKE_SOURCE_DIR}/../../mcu_ws/install/include)
    set_target_properties(${APPNAME} PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${BUILD_LIB}"
        LIBRARY_OUTPUT_DIRECTORY "${BUILD_LIB}"
        RUNTIME_OUTPUT_DIRECTORY "${BUILD_BIN}")
    target_link_libraries(${APPNAME} LINK_PUBLIC
                          ${LIBFREERTOS} ${LIBFREERTOSLINUX}
                          ${EXE_LINK_LIBS} ${CMAKE_SOURCE_DIR}/build/libmicroros.a)
endfunction(BuildTestApp)

set(MICROROSAPP "microrosapp")
set(MICROROSAPP_FILES
        # "${CMAKE_SOURCE_DIR}/allocators.c"
        ${CMAKE_SOURCE_DIR}/main.c
        # ${UROS_APP_FOLDER}/app.c
        )
BuildTestApp(${MICROROSAPP} ${MICROROSAPP_FILES})

# add_executable(${MICROROSAPP} ${MICROROSAPP_FILES})
# target_include_directories(${MICROROSAPP} SYSTEM PRIVATE ${FREERTOS_INCLUDE})
# target_include_directories(${MICROROSAPP} PUBLIC ${FREERTOS_SOURCE})
# target_include_directories(${MICROROSAPP} PUBLIC ${PATH_INCLUDE})
# target_include_directories(${MICROROSAPP} PUBLIC ${PATH_PORTABLE})
# target_include_directories(${MICROROSAPP} PUBLIC ${CMAKE_SOURCE_DIR}/../../mcu_ws/install/include)
# set_target_properties(${MICROROSAPP} PROPERTIES
#     ARCHIVE_OUTPUT_DIRECTORY "${BUILD_LIB}"
#     LIBRARY_OUTPUT_DIRECTORY "${BUILD_LIB}"
#     RUNTIME_OUTPUT_DIRECTORY "${BUILD_BIN}")
# target_link_libraries(${MICROROSAPP} LINK_PUBLIC
#                         ${LIBFREERTOS} ${LIBFREERTOSLINUX}
#                         ${EXE_LINK_LIBS} ${CMAKE_SOURCE_DIR}/build/libmicroros.a)

