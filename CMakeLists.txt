cmake_minimum_required(VERSION 3.16)

project(GRIPPER_SDK VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gripper_sdk
    src/gripper_sdk.cpp
)

target_include_directories(gripper_sdk
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

add_executable(example_main
    examples/example_main.cpp
)

target_link_libraries(example_main
    PRIVATE gripper_sdk
)