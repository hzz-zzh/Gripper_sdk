cmake_minimum_required(VERSION 3.16)

project(GRIPPER_SDK VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(gripper_sdk
    src/core/gripper.cpp
    src/core/gripper_device.cpp
    src/c_api/gripper_c_api.cpp
    src/protocol/protocol.cpp
    src/protocol/crc.cpp
    src/transport/rs485_transport.cpp
)

target_include_directories(gripper_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(gripper_sdk PUBLIC cxx_std_17)

add_executable(example_c_api
    examples/example_c_api.c
)

add_executable(example_homing_c_api
    examples/example_homing_c_api.c
)

target_link_libraries(example_homing_c_api
    PRIVATE gripper_sdk
)

target_include_directories(example_homing_c_api
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 如果你目前 gripper_sdk 还没有把这些依赖正确向外传递，
# 先临时补上，避免静态链接时报错
if(UNIX AND NOT APPLE)
    target_link_libraries(example_homing_c_api PRIVATE stdc++ m)
endif()

target_link_libraries(example_c_api
    PRIVATE gripper_sdk
)

install(TARGETS gripper_sdk
    EXPORT GripperSDKTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
    inc/c_api/gripper_c_api.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gripper
)

install(FILES
    examples/example_c_api.c
    DESTINATION ${CMAKE_INSTALL_DATADIR}/gripper_sdk/examples
)

install(EXPORT GripperSDKTargets
    FILE GripperSDKTargets.cmake
    NAMESPACE Gripper::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/GripperSDK
)

set_target_properties(example_c_api PROPERTIES LINKER_LANGUAGE CXX)