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

# source files
set(LIB_SOURCE_FILES
      "${FREERTOS_SOURCE}/tasks.c"
      "${FREERTOS_SOURCE}/list.c"
      "${FREERTOS_SOURCE}/queue.c"
      "${FREERTOS_SOURCE}/croutine.c"
      "${FREERTOS_SOURCE}/portable/MemMang/heap_3.c"
      )

# library name
set(LIBFREERTOS freertos)

add_library(${LIBFREERTOS} ${LIB_SOURCE_FILES})
target_include_directories(${LIBFREERTOS} SYSTEM PRIVATE ${FREERTOS_INCLUDE})
target_include_directories(${LIBFREERTOS} PUBLIC ${FREERTOS_SOURCE})
target_include_directories(${LIBFREERTOS} PUBLIC ${PATH_INCLUDE})
target_include_directories(${LIBFREERTOS} PUBLIC ${PATH_PORTABLE})
set_target_properties(${LIBFREERTOS} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${BUILD_LIB}"
    LIBRARY_OUTPUT_DIRECTORY "${BUILD_LIB}"
    RUNTIME_OUTPUT_DIRECTORY "${BUILD_BIN}")
