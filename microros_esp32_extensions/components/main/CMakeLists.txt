file(GLOB SRCS ${UROS_APP_FOLDER}/*.c*)
idf_component_register(SRCS main.c ${SRCS}
                       INCLUDE_DIRS ".")

set(MICROROS_LIB_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/../../../../mcu_ws/install)
set(MICROROS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../mcu_ws/install/include)

set(microros_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/../../)
set(microros_build_dir ${CMAKE_CURRENT_BINARY_DIR}/microros_esp32_extensions)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "$(MAKE)")
else()
set(submake "make")
endif()

externalproject_add(microroslib_build
    PREFIX     ${microros_build_dir}    # Root dir for entire project
    SOURCE_DIR ${microros_src_dir}
    BINARY_DIR ${microros_src_dir}      # This particular build system is invoked from the root
    CONFIGURE_COMMAND ""                # Skip configuring the project, e.g. with autoconf
    BUILD_COMMAND
        ${submake}
            PREFIX=${microros_src_dir}
            CC=${CMAKE_C_COMPILER}
            AR=${CMAKE_AR}
            CFLAGS=${CMAKE_C_FLAGS}
            CXX=${CMAKE_CXX_COMPILER}
            CXXFLAGS=${CMAKE_CXX_FLAGS}
            IDF_PATH=${IDF_PATH}
            APP_META=${UROS_APP_FOLDER}/app-colcon.meta

    INSTALL_COMMAND ""      # This particular build system has no install command
    BUILD_BYPRODUCTS ${MICROROS_LIB_DIR}/libmicroros.a
    )

file(MAKE_DIRECTORY ${MICROROS_LIB_DIR})
file(MAKE_DIRECTORY ${MICROROS_INCLUDE_DIR})

add_library(microroslib STATIC IMPORTED GLOBAL)
add_dependencies(microroslib microroslib_build idf::libatomic)
target_link_libraries(microroslib INTERFACE idf::libatomic)

set_target_properties(microroslib PROPERTIES IMPORTED_LOCATION             ${MICROROS_LIB_DIR}/libmicroros.a)
set_target_properties(microroslib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MICROROS_INCLUDE_DIR})

add_dependencies(${COMPONENT_LIB} microroslib)
target_include_directories(${COMPONENT_LIB} PUBLIC microroslib)
target_link_libraries(${COMPONENT_LIB} PUBLIC microroslib)
