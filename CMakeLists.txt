cmake_minimum_required(VERSION 3.16)

project(GRIPPER_SDK VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(gripper_sdk
    src/core/gripper.cpp
    src/protocol/protocol.cpp
    src/protocol/crc.cpp
    src/transport/rs485_transport.cpp
)

target_include_directories(gripper_sdk
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(gripper_sdk PUBLIC cxx_std_17)

add_executable(example_main
    examples/example_main.cpp
)

target_link_libraries(example_main
    PRIVATE gripper_sdk
)

target_include_directories(example_main
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)