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
    set_target_properties(${APPNAME} PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${BUILD_LIB}"
        LIBRARY_OUTPUT_DIRECTORY "${BUILD_LIB}"
        RUNTIME_OUTPUT_DIRECTORY "${BUILD_BIN}")
    target_link_libraries(${APPNAME} LINK_PUBLIC
                          ${LIBFREERTOS} ${LIBFREERTOSLINUX}
                          ${EXE_LINK_LIBS})
endfunction(BuildTestApp)

# linux test program 01
set(LINUXTES01 "linuxtest01")
set(TEST01_FILES
      "${PATH_TESTS}/linuxtest01.c")
BuildTestApp(${LINUXTES01} ${TEST01_FILES})

# linux test program 02
set(LINUXTES02 "linuxtest02")
set(TEST02_FILES
      "${PATH_TESTS}/linuxtest02.c")
BuildTestApp(${LINUXTES02} ${TEST02_FILES})
