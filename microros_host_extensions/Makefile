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

OUTPUT_ROOT     := "build"

BUILDTYPE		?= "debug"

ifeq ($(BUILDTYPE),release)
else
  BUILDTYPE = debug
endif

OUTPUT_ROOT     := "build/x86-$(BUILDTYPE)"
CMAKE_FOLDER    := "./$(OUTPUT_ROOT)/cmake"
CMAKE_DEFINES   := "-DCMAKE_TOOLCHAIN_FILE=./cmake/toolchain/toolchain_linux-i686.cmake"
CMAKE_TYPE      := "-DCMAKE_BUILD_TYPE=$(BUILDTYPE)"

.phony: all clean

all: libmicroros
	git submodule update --recursive
	mkdir -p $(CMAKE_FOLDER)
	cmake -B$(CMAKE_FOLDER) -H./ $(CMAKE_DEFINES) $(CMAKE_TYPE)
	make -C $(CMAKE_FOLDER)

clean:
	rm -rf $(OUTPUT_ROOT)

#######################################
# build micro-ROS
#######################################

colcon_compile:
	cd $(UROS_DIR); \
	colcon build \
		--packages-ignore-regex=.*_cpp \
		--metas $(UROS_DIR)/colcon.meta $(UROS_APP_FOLDER)/app-colcon.meta \
		--cmake-args \
		-DCMAKE_POSITION_INDEPENDENT_CODE=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/arm_toolchain.cmake \
		-DCMAKE_VERBOSE_MAKEFILE=ON; \

libmicroros: colcon_compile
	mkdir -p $(UROS_DIR)/libmicroros; cd $(UROS_DIR)/libmicroros; \
	for file in $$(find $(UROS_DIR)/install/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; ar x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	ar rc libmicroros.a *.obj; mkdir -p $(BUILD_DIR); cp libmicroros.a $(BUILD_DIR); ranlib $(BUILD_DIR)/libmicroros.a; \
	cd ..; rm -rf libmicroros;
