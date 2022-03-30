file(GLOB SRCS ${UROS_APP_FOLDER}/*.c*)
idf_component_register(SRCS main.c microros_transports.c ${SRCS}
                       INCLUDE_DIRS ".")

set(EXTENSIONS_DIR ${COMPONENT_DIR}/..)
set(UROS_DIR       ${COMPONENT_DIR}/../../../mcu_ws)

set(MICROROS_INCLUDE_DIR ${UROS_DIR}/install/include)
set(MICROROS_PREFIX ${CMAKE_BINARY_DIR}/libmicroros-prefix)


if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "$(MAKE)")
else()
set(submake "make")
endif()

externalproject_add(libmicroros
    PREFIX     ${MICROROS_PREFIX}
    SOURCE_DIR ${UROS_DIR}
    BINARY_DIR ${EXTENSIONS_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND
        ${submake} -f libmicroros.mk
            CC=${CMAKE_C_COMPILER}
            AR=${CMAKE_AR}
            CFLAGS=${CMAKE_C_FLAGS}
            CXX=${CMAKE_CXX_COMPILER}
            CXXFLAGS=${CMAKE_CXX_FLAGS}
            BUILD_DIR=${CMAKE_BINARY_DIR}
            IDF_PATH=${IDF_PATH}
            APP_META=${UROS_APP_FOLDER}/app-colcon.meta
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/libmicroros.a
    )

file(MAKE_DIRECTORY ${MICROROS_INCLUDE_DIR})

add_library(microros STATIC IMPORTED GLOBAL)
add_dependencies(microros libmicroros)
target_link_libraries(microros INTERFACE c idf::lwip)

set_target_properties(microros PROPERTIES IMPORTED_LOCATION             ${CMAKE_BINARY_DIR}/libmicroros.a)
set_target_properties(microros PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MICROROS_INCLUDE_DIR})

execute_process(
    COMMAND
        make -f libmicroros.mk get_package_names
    WORKING_DIRECTORY
        ${EXTENSIONS_DIR}
    OUTPUT_VARIABLE
        INCLUDE_ROS2_PACKAGES
    )

foreach(pkg ${INCLUDE_ROS2_PACKAGES})
    file(MAKE_DIRECTORY ${MICROROS_INCLUDE_DIR}/${pkg})
    target_include_directories(microros INTERFACE ${MICROROS_INCLUDE_DIR}/${pkg})
endforeach()

set_directory_properties( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${UROS_DIR}/install;${UROS_DIR}/build;${UROS_DIR}/log" )

add_dependencies(${COMPONENT_LIB} microros)
target_include_directories(${COMPONENT_LIB} PUBLIC microros)
target_link_libraries(${COMPONENT_LIB} PUBLIC microros)
