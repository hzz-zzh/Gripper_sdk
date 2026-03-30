# Gripper SDK

基于 RS485 自定义协议的夹爪控制 SDK。  
当前版本对外发布 **C API**，内部实现使用 C++。

## 1. 功能简介

SDK 当前提供以下主要接口：

- 连接/断开设备
- 初始化夹爪零点
- 打开夹爪
- 关闭夹爪
- 停止夹爪
- 按目标位置控制
- 按百分比控制夹爪开合

主要 C API：

- `gripper_create`
- `gripper_destroy`
- `gripper_connect`
- `gripper_disconnect`
- `gripper_initialize`
- `gripper_open`
- `gripper_close`
- `gripper_stop`
- `gripper_move_to_position`
- `gripper_move_to_position_with_limits`
- `gripper_move_to_percent`
- `gripper_get_last_error`

## 2. 平台说明

当前支持平台：

- Linux

串口通信基于 RS485，自定义协议，小端字节序，CRC16_MODBUS 校验。

## 3. 工程目录

```text
GRIPPER_SDK/
├── inc/
│   └── c_api/
│       └── gripper_c_api.h
├── src/
│   ├── c_api/
│   ├── core/
│   ├── protocol/
│   └── transport/
├── examples/
├── cmake/
├── CMakeLists.txt
└── README.md