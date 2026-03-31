cmake_minimum_required(VERSION 3.16)

project(GRIPPER_SDK VERSION 0.1.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(GRIPPER_SDK_BUILD_EXAMPLES "Build SDK examples" ON)

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

add_library(Gripper::gripper_sdk ALIAS gripper_sdk)

target_include_directories(gripper_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(gripper_sdk PUBLIC cxx_std_17)

set_target_properties(gripper_sdk PROPERTIES
    OUTPUT_NAME gripper_sdk
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

if(GRIPPER_SDK_BUILD_EXAMPLES)

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/example_c_api.c")
        add_executable(example_c_api
            examples/example_c_api.c
        )

        target_link_libraries(example_c_api
            PRIVATE Gripper::gripper_sdk
        )

        target_include_directories(example_c_api
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc
        )

        set_target_properties(example_c_api PROPERTIES LINKER_LANGUAGE CXX)
    endif()
endif()

install(TARGETS gripper_sdk
    EXPORT GripperSDKTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 只安装对外 C API 头文件
install(FILES
    inc/c_api/gripper_c_api.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gripper
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/example_c_api.c")
    install(FILES
        examples/example_c_api.c
        DESTINATION ${CMAKE_INSTALL_DATADIR}/gripper_sdk/examples
    )
endif()

# 导出 targets
install(EXPORT GripperSDKTargets
    FILE GripperSDKTargets.cmake
    NAMESPACE Gripper::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/GripperSDK
)

# 生成 Config / ConfigVersion
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GripperSDKConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/GripperSDKConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/GripperSDK
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/GripperSDKConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/GripperSDKConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/GripperSDKConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/GripperSDK
)