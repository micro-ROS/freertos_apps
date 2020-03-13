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


# freertoslinux library
set(LIBFREERTOSLINUX "freertoslinux")

# source files
set(LIBLINUX_SOURCE_FILES
      "${PATH_PORTABLE}/port.c"
      "${PATH_PORTABLE}/partest.c"
      )

add_library(${LIBFREERTOSLINUX} ${LIBLINUX_SOURCE_FILES})
target_include_directories(${LIBFREERTOSLINUX} SYSTEM PRIVATE ${FREERTOS_INCLUDE})
target_include_directories(${LIBFREERTOSLINUX} PUBLIC ${FREERTOS_SOURCE})
target_include_directories(${LIBFREERTOSLINUX} PUBLIC ${PATH_INCLUDE})
target_include_directories(${LIBFREERTOSLINUX} PUBLIC ${PATH_PORTABLE})
set_target_properties(${LIBFREERTOSLINUX} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BUILD_LIB}"
    LIBRARY_OUTPUT_DIRECTORY "${BUILD_LIB}"
    RUNTIME_OUTPUT_DIRECTORY "${BUILD_BIN}")
